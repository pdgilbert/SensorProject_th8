//! Measure temperature and humidity on eight AHT10 or AHT20 sensors on I2C1, multiplexed with crate 
//! (The aht20-driver crate seems to work also with aht10 sensors.)
//! TESTING WITH EARLY PROTOTYPE  sensors on I2C1,  SSD1306 on I2C2.
//! CONSIDER REVERSE AS ON T16
//! Using xcs9548a because all sensors use the same address.
//! (Optionally) Display using SSD1306 on I2C2.
//! Transmit with LoRa on SPI.

//! Status: (August 2025) 
//!        Working with hardware multiplexI2C v0.1.0 2024-09 and blackpill stm32f401 and stm32f411
//!         using AHT10 sensors.

//!  To Do:
//!    - decide I2C1 or I2C2 for ssd or sensors.
//!    - optional display could be detected rather than indicated by feature no_ssd_display.

// Compile the binary choosing features st32f401 or stm32f411 and optionally no_ssd_display.
// MONITOR_ID="whatever" cargo build ...
// MONITOR_ID="whatever"  cargo  run --no-default-features --target thumbv7em-none-eabihf --features stm32f401 --bin th8
// MONITOR_ID="whatever"  cargo  run --no-default-features --target thumbv7em-none-eabihf --features no_ssd_display,stm32f401 --bin th8
// or  cargo:rustc-env=MONITOR_ID="whatever"

///////////////////// 

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

const MONITOR_ID: &str = option_env!("MONITOR_ID").expect("THxx");
const MONITOR_IDU : &[u8] = MONITOR_ID.as_bytes();

const MODULE_CODE:  &str = "th8";
const READ_INTERVAL:  u32 = 600; // used as seconds 
const BLINK_DURATION: u32 = 40;   // used as ms
const S_FMT:       usize  = 14;
const MESSAGE_LEN: usize  = 16 * S_FMT;  

//////////////////// 

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::hprintln;   // this should be commented out to run without semihosting
//use cortex_m::asm; // for delay
use cortex_m_rt::entry;

use core::fmt::Write;

use stm32f4xx_hal::{
    pac::{Peripherals, I2C1, I2C2, SPI1, TIM5},
    rcc::{RccExt},
    spi::{Spi},
    i2c::I2c,   //this is a type vs  embedded_hal::i2c::I2c trait
    gpio::{Output, PushPull, GpioExt, Input},
    gpio::{Pin}, 
    gpio::{gpioa::{PA1, PA4, }},
    gpio::{gpiob::{PB4, PB5, }},  
    gpio::{gpioc::{PC13}},
    prelude::*,
    //block,
    timer::{Delay as halDelay},
    timer::{TimerExt},
};

use embedded_hal::{spi::{Mode, Phase, Polarity},
                   delay::DelayNs,
                   digital::OutputPin,
};


//////////////////////  ssd display  

// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html re fonts

use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, // need iso for degree symbol
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

//common display sizes are 128x64 and 128x32
type  DisplaySizeType = ssd1306::prelude::DisplaySize128x64;
const DISPLAYSIZE: DisplaySizeType = DisplaySize128x64;
type  DisplayType = Ssd1306<I2CInterface<I2c<I2C2>>, DisplaySizeType, BufferedGraphicsMode<DisplaySizeType>>;

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};


/////////////////////  lora  //////////////////////////////////

use th8::lora::Base;
use th8::lora::{CONFIG_RADIO};

use radio_sx127x::{
    Transmit,  // trait needs to be in scope to find  methods start_transmit and check_transmit.
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
    //device::regs::Register, // for config examination on debugging
    // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,

};


//////////////////////  sensor crate  //////////////////////////////////

use aht20_driver; 
use aht20_driver::{AHT20, AHT20Initialized, SENSOR_ADDRESS as S_ADDR}; 


//////////////////////  xca  //////////////////////////////////
//use xca9548a::{Read, Write, WriteRead};
use xca9548a::{SlaveAddr as XcaSlaveAddr, Xca9548a, I2cSlave}; 


//   //////////////////////////////////////////////////////////////////////

trait LED: OutputPin {  // see The Rust Programming Language, section 19, Using Supertraits...
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

}

impl LED for  PC13<Output<PushPull>> {}    

struct SpiExt {  cs:    PA4<Output<PushPull>>, 
                 busy:  PB4<Input<>>, 
                 ready: PB5<Input<>>, 
                 reset: PA1<Output<PushPull>>
}

const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

//use th8::setup::{setup_from_dp, LED, Delay1Type};


/////////////////////////// functions ////////////////////////////////////////

    fn show_display(
        th: [(f32, f32); 8],
        disp: &mut Option<DisplayType>,
    ) -> ()
    {    
     if disp.is_some() { // show_message does nothing if disp is None. 
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines

       // Note: The screen layout accommodates no more than 4 sensors installed! 
       // If more are installed then the code will probably panic 
       // trying to write beyond the limit of the screen variable.
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line,   "{:.0}°{:.0}% {:.0}°{:.0}%\n",  th[0].0, th[0].1,  th[1].0, th[1].1 ).unwrap();
       write!(line,   "{:.0}°{:.0}% {:.0}°{:.0}%\n",  th[2].0, th[2].1,  th[3].0, th[3].1 ).unwrap();
       write!(line,   "{:.0}°{:.0}% {:.0}°{:.0}%\n",  th[4].0, th[4].1,  th[5].0, th[5].1 ).unwrap();
       write!(line,   "{:.0}°{:.0}% {:.0}°{:.0}%\n",  th[6].0, th[6].1,  th[7].0, th[7].1 ).unwrap();
    //too long   write!(line,   "{:.1}°C {:.0}%RH {}°C {}%RH\n",  th[6].0, th[6].1,  th[7].0, th[7].1 ).unwrap();

   // CHECK SIGN IS CORRECT FOR -0.3 C

       show_message(&line, disp);
       ()
     };
    }



    fn show_message(
        text: &str,   //text_style: MonoTextStyle<BinaryColor>,
        //disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        disp: &mut Option<DisplayType>,
    ) -> ()
    {
       if disp.is_some() { // show_message does nothing if disp is None. 
           //let mut disp = *disp;   
           // workaround. build here because text_style cannot be shared
           let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
        
           disp.as_mut().expect("REASON").clear_buffer();
           Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
                   .draw(disp.as_mut().expect("REASON"))
                   .unwrap();

           disp.as_mut().expect("REASON").flush().unwrap();
       };
       ()
    }

    fn form_message(
            th: [(f32, f32); 8],
           ) -> heapless::Vec<u8, MESSAGE_LEN> {

        let mut line: heapless::Vec<u8, MESSAGE_LEN> = heapless::Vec::new(); 
        let mut zz: heapless::Vec<u8, S_FMT> = heapless::Vec::new(); 

        // Consider handling error in next. If line is too short then attempt to write it crashes
        
        for i in 0..MONITOR_IDU.len() { line.push(MONITOR_IDU[i]).unwrap()};
        line.push(b'<').unwrap();
        
        for i in 0..th.len() {
                //hprintln!("xJ{}:({:.1},{:.0})x",  i+1, th[i].0, th[i].1); 
                zz.clear();
                write!(zz,  "J{}:({:.1},{:.0})",  i+1, th[i].0, th[i].1).unwrap(); // must not exceed S_FMT
                for j in 0..zz.len() {line.push(zz[j]).unwrap()};
        };

        line.push(b'>').unwrap();
        //hprintln!("{:?}", line);

        line
     }

    //type LoraType = Sx127x<Base<Spi<SPI1>, Pin<'A', 4, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 0, Output>, halDelay<TIM5, 1000000>>>;
    type LoraType = Sx127x<Base<Spi<SPI1>, Pin<'A', 4, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 1, Output>, halDelay<TIM5, 1000000>>>;

    fn send(
            lora: &mut LoraType,
            m:  heapless::Vec<u8, MESSAGE_LEN>,
            disp: &mut Option<DisplayType>,
           ) -> Result<bool, bool>{
        match lora.start_transmit(&m) {
            Ok(_b)   => {//show_message("start_transmit ok", disp);
                         //hprintln!("lora.start ok").unwrap()
                        } 
            Err(_e)  => {show_message("start_transmit error", disp);
                         //hprintln!("Error in lora.start_transmit()").unwrap()
                        }
        };

        lora.delay_ms(10); // treated as seconds. Without some delay next returns bad. (interrupt may also be an option)

        // return check_transmit() result could be returned, but this simplifies the reurn type from
        // Result<bool, radio_sx127x::Error<HalError<stm32f4xx_hal::spi::Error, Infallible, Infallible>>>   
        let r = lora.check_transmit();
        match r { Ok(_b)   => {Result::Ok(true)}, Err(_e) => {Result::Err(false)} }
    }

//////////////////////////  main  /////////////////////////////////////

#[entry]
fn main() -> ! {

    //hprintln!("th8");

    let dp = Peripherals::take().unwrap();


    let mut rcc = dp.RCC.constrain();
    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); // not pb6, should be pb7 or pb9 on pins 43 or 46
    let i2c1 = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &mut rcc);

    let scl = gpiob.pb10.into_alternate_open_drain();
    let sda = gpiob.pb3.into_alternate_open_drain();
    let i2c2 = I2c::new(dp.I2C2, (scl, sda), 400.kHz(), &mut rcc);

    let mut led = gpioc.pc13.into_push_pull_output();
    led.off();

    let spi = Spi::new(
        dp.SPI1,
        (
            Some(gpioa.pa5.into_alternate()), // sck  
            Some(gpioa.pa6.into_alternate()), // miso 
            Some(gpioa.pa7.into_alternate()), // mosi 
        ),
        MODE, 8.MHz(), &mut rcc,
    );
    
    let spiext = SpiExt {
         cs:    gpioa.pa4.into_push_pull_output(), //CsPin         
         busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
         ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
         reset: gpioa.pa1.into_push_pull_output(), //ResetPin   
         };   


    let mut delay = dp.TIM2.delay::<1000000>(&mut rcc);
    let delay_for_lora = dp.TIM5.delay(&mut rcc);

    led.off();
    delay.delay_ms(2000); // treated as ms
    led.on();
    delay.delay_ms(BLINK_DURATION); // treated as ms
    led.off();

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C
    
    let mut z = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0).into_buffered_graphics_mode();

    //hprintln!("display initializing.");

    // disp is Optional because ssd screen may not be plugged in. 
    // It would be nice if it could set None if init fails, but so far not luck, so use cfg
    // If it is None then messages displays are skipped.


    #[cfg(feature = "no_ssd_display")]
    let mut display: Option<DisplayType> = None;

    #[cfg(not(feature = "no_ssd_display"))]
    let mut display: Option<DisplayType> = match z.init() {
        Ok(_d)  => {Some(z)} 
        Err(_e) => {None}
    };

    // let mut screen: [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES] = [R_VAL; DISPLAY_LINES];

    show_message(MODULE_CODE, &mut display);

    delay.delay_ms(2000); // treated as ms
    //hprintln!("display init done.");

   
    /////////////////////  xca   multiple devices on i2c bus

    let switch1 = Xca9548a::new(i2c1, XcaSlaveAddr::default());

    show_message(&"Sensors on xca", &mut display);

    /////////////////////  sensors on xca    // Start the sensors.

    type I2c1Type = I2c<I2C1>;
    type SensType<'a> = AHT20Initialized<'a, I2cSlave<'a, Xca9548a<I2c1Type>, I2c1Type>>;

    //  Option allows for the possibility that some sensors are missing.
    //const gives this `static lifetime, which causes does not live long enough here
    //const SENSER: Option::<SensType> = None; 
    //let mut sensors: [Option<SensType>; 8] = [SENSER; 8];
    
    let mut sensors: [Option<SensType>; 8] = [ None, None, None, None, None, None, None, None, ];

    let mut th: [(f32, f32); 8] = [(-50.0, -50.0); 8];  //default values for no sensor

    // Split the device and pass the virtual I2C devices to sensor driver
    let switch1parts = switch1.split();

    //  a loop for this should be possible, but my attempts cause lifetime problems.

    let mut z = AHT20::new(switch1parts.i2c0,  S_ADDR);
    sensors[0] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
 
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c1,  S_ADDR);
    sensors[1] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c2,  S_ADDR);
    sensors[2] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c3,  S_ADDR);
    sensors[3] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c4,  S_ADDR);
    sensors[4] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c5,  S_ADDR);
    sensors[5] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c6,  S_ADDR);
    sensors[6] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    let mut z = AHT20::new(switch1parts.i2c7,  S_ADDR);
    sensors[7] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_e) => {None} };
    delay.delay_ms(500);

    /////////////////  don't need this if there is no screen
    //hprintln!("Sensors in use:");
    show_message("Sensors in use:", &mut display);

    led.off();
    delay.delay_ms(2000); // treated as ms   
    led.on();
    delay.delay_ms(BLINK_DURATION); // treated as ms
    led.off();

//    for  i in 0..7 {  // 7 is sensors.len(() 
//       if  sensors[i].is_some() {led.blink(50, &mut delay)}
//       else {led.blink(1000, &mut delay)};
//    };


//    screen[0].clear();
//    screen[1].clear();
//    write!(screen[0], "Sensors in use:").unwrap();
//
//    for  i in 0..7 {  // 7 is sensors.len(() 
//       if  sensors[i].is_some() {write!(screen[1], "{} ", i+1).unwrap()}
//    };
//
//    show_screen(&screen, &mut display);
//    delay.delay_ms(5000);
//
//    screen[0].clear();
//    write!(screen[0], "   °C   %RH").unwrap();
//
//    /////////////////

    /////////////////////   lora
    //hprintln!("lora");
    
    // cs is named nss on many radio_sx127x module boards
    let z = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay_for_lora, 
                   &CONFIG_RADIO ); 

    let mut lora =  match z {
        Ok(lr)  => {show_message("lora setup ok", &mut display);
                    //hprintln!("lora setup completed.").unwrap();
                    lr
                   } 
        Err(e)  => {show_message("lora setup Error", &mut display);
                    //hprintln!("Error in lora setup. {:?}", e).unwrap();
                    //asm::bkpt();
                    panic!("{:?}", e) 
                   }
    };
 

    //delay consumed by lora. It is available in lora BUT treats arg as seconds not ms!!

    //led.blink(1000, &mut delay2);
    //delay2.delay_ms(500); 
    lora.delay_ms(1);  // arg is being treated as seconds

    //hprintln!("loop");
    
    loop {
      // blink
      led.on(); 
      delay.delay_ms(BLINK_DURATION); 
      led.off();
  
      for i in 0..7 { // 7 is sensors.len(() 
         // hprintln!("sensor {}", i);
         th[i] =  match   &mut sensors[i] {
                                 Some(s) => {let v = s.measure(&mut delay).unwrap();// uses a DelayNs
                                             (v.temperature, v.humidity)
                                            },  
                                 None    => {(-50.0, -50.0)}, // default to transmit for no sensor 
                                };
      };



//      let mut ln = 1;  // screen line to write. rolls if number of sensors exceed DISPLAY_LINES

      show_display(th, &mut display);

      let message = form_message(th);
      //hprintln!("message {:?}", message);
      //hprintln!("send message");
      let r = send(&mut lora, message, &mut display);
      delay.delay_ms(3000);// delay here leave temp/humidity on screen longer
      
      match r {
            Ok(b)   => {if b {show_message("TX good", &mut display);
                              //hprintln!("TX good").unwrap(); 
                             }
                        else {show_message("TX bad", &mut display);
                              //hprintln!("TX bad").unwrap()
                             }
                       }
            Err(_e) => {show_message("transmit failed", &mut display);
                        //hprintln!("check_transmit() Error. Should return True or False.").unwrap()
                       }
        };
 
      //delay.delay_ms(READ_INTERVAL);// treated as ms
      lora.delay_ms(READ_INTERVAL);  // treated as seconds
    }
}
