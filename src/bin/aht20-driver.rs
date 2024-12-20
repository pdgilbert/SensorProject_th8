//! Measure temperature and humidity on eight AHT20 sensors on I2C1, multiplexed with crate 
//! xcs9548a because all sensors use the same address.
//! Display using SSD1306 on I2C2. Transmit with LoRa on SPI.

//!  To Do:
//!    - make work better.

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

///////////////////// 

// set this with
// MONITOR_ID="whatever" cargo build ...
// or  cargo:rustc-env=MONITOR_ID="whatever"

const MONITOR_ID: &str = option_env!("MONITOR_ID").expect("Txxx");
const MONITOR_IDU : &[u8] = MONITOR_ID.as_bytes();

const MODULE_CODE:  &str = "th8-f401 aht20"; //"th8-f401"; 
const READ_INTERVAL:  u32 = 300;  // used as seconds  but 
const BLINK_DURATION: u32 = 1;  // used as seconds  but  ms would be better
const S_FMT:       usize  = 12;
const MESSAGE_LEN: usize  = 16 * S_FMT;  


//////////////////// 

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use cortex_m_semihosting::{debug, hprintln};
use cortex_m_semihosting::{hprintln};
use cortex_m::asm; // for delay
use cortex_m_rt::entry;

use core::fmt::Write;
use embedded_hal::delay::DelayNs;

/////////////////////  sensor crate

use aht20_driver; 
use aht20_driver::{AHT20, AHT20Initialized, SENSOR_ADDRESS as S_ADDR} ; 

/////////////////////

use stm32f4xx_hal::{
    pac::{Peripherals, I2C1, I2C2, SPI1, TIM5},
    timer::{Delay as halDelay},
    rcc::{RccExt},
    spi::{Spi},
    i2c::I2c as I2cType,   //this is a type vs  embedded_hal::i2c::I2c trait
    gpio::{Output, PushPull, GpioExt, Input},
    prelude::*,
    timer::{TimerExt},
    gpio::{Pin}, 
    gpio::{gpioa::{PA0, PA4, }},
    gpio::{gpiob::{PB4, PB5, }},
    gpio::{gpioc::{PC13}},
};


use embedded_hal::{spi::{Mode, Phase, Polarity},digital::OutputPin,};
                   //i2c::{I2c},   //trait
                   //delay::DelayNs,
                   


/////////////////////   ssd
// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html re fonts

use embedded_graphics::{
    mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

type  DisplaySize = ssd1306::prelude::DisplaySize128x64;
type  DisplayType = Ssd1306<I2CInterface<I2cType<I2C2>>, DisplaySize, BufferedGraphicsMode<DisplaySize>>;

//common display sizes are 128x64 and 128x32
const DISPLAYSIZE: DisplaySize = DisplaySize128x64;

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

/////////////////////  xca
//use xca9548a::{Read, Write, WriteRead};

use xca9548a::{SlaveAddr as XcaSlaveAddr, Xca9548a, I2cSlave}; 



/////////////////////  lora

use th8_f401::lora::Base;
use th8_f401::lora::{CONFIG_RADIO};

use radio_sx127x::{
    Transmit,  // trait needs to be in scope to find  methods start_transmit and check_transmit.
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
    //device::regs::Register, // for config examination on debugging
    // read_register, get_mode, get_signal_bandwidth, get_coding_rate_4, get_spreading_factor,

};


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
                 reset: PA0<Output<PushPull>>
}

const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

//   //////////////////////////////////////////////////////////////////

    fn show_display(
        th: [(f32, f32); 8],
        disp: &mut Option<DisplayType>,
    ) -> ()
    {    
     if disp.is_some() { // show_message does nothing if disp is None. 
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line,   "{}°C {}%RH {}°C {}%RH\n",  th[0].0, th[0].1,  th[1].0, th[1].1 ).unwrap();
       write!(line,   "{}°C {}%RH {}°C {}%RH\n",  th[2].0, th[2].1,  th[3].0, th[3].1 ).unwrap();
       write!(line,   "{}°C {}%RH {}°C {}%RH\n",  th[4].0, th[4].1,  th[5].0, th[5].1 ).unwrap();
       write!(line,   "{}°C {}%RH {}°C {}%RH",  th[6].0, th[6].1,  th[7].0, th[7].1 ).unwrap();

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
                zz.clear();
                write!(zz,  " J{}:({},{})",  i+1, th[i].0, th[i].1).unwrap(); // must not exceed S_FMT
                for j in 0..zz.len() {line.push(zz[j]).unwrap()};
        };

        line.push(b'>').unwrap();
        hprintln!("{:?}", line);

        line
     }


    type LoraType = Sx127x<Base<Spi<SPI1>, Pin<'A', 4, Output>, Pin<'B', 4>, Pin<'B', 5>, Pin<'A', 0, Output>, halDelay<TIM5, 1000000>>>;

    fn send(
            lora: &mut LoraType,
            m:  heapless::Vec<u8, MESSAGE_LEN>,
            disp: &mut Option<DisplayType>,
           ) -> () {
        
        match lora.start_transmit(&m) {
            Ok(_b)   => {//show_message("start_transmit ok", disp);
                         //hprintln!("lora.start ok").unwrap()
                        } 
            Err(_e)  => {show_message("start_transmit error", disp);
                         //hprintln!("Error in lora.start_transmit()").unwrap()
                        }
        };

        lora.delay_ms(10); // treated as seconds. Without some delay next returns bad. (interrupt may also be an option)

        match lora.check_transmit() {
            Ok(b)   => {if b {show_message("TX good", disp);
                              //hprintln!("TX good").unwrap(); 
                             }
                        else {show_message("TX bad", disp);
                              //hprintln!("TX bad").unwrap()
                             }
                       }
            Err(_e) => {show_message("check_transmit Fail", disp);
                        //hprintln!("check_transmit() Error. Should return True or False.").unwrap()
                       }
        };
       ()
    }

//   //////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {

   //hprintln!("t8-th4-f401").unwrap();

   let dp = Peripherals::take().unwrap();
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   // according to  https://github.com/rtic-rs/rtic/blob/master/examples/stm32f411_rtc_interrupt/src/main.rs
   // 25 MHz must be used for HSE on the Blackpill-STM32F411CE board according to manual
   // let clocks = rcc.cfgr.use_hse(25.MHz()).freeze();
   
   let gpioa = dp.GPIOA.split();
   let gpiob = dp.GPIOB.split();
   let gpioc   = dp.GPIOC.split();

   let scl = gpiob.pb6.into_alternate_open_drain(); 
   let sda = gpiob.pb7.into_alternate_open_drain(); 
   let i2c1 = I2cType::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

   let scl = gpiob.pb10.into_alternate_open_drain();
   let sda = gpiob.pb3.into_alternate_open_drain();
   let i2c2 = I2cType::new(dp.I2C2, (scl, sda), 400.kHz(), &clocks);

   let mut led = gpioc.pc13.into_push_pull_output();
   led.off();

   let spi = Spi::new(
       dp.SPI1,
       (
           gpioa.pa5.into_alternate(), // sck  
           gpioa.pa6.into_alternate(), // miso 
           gpioa.pa7.into_alternate(), // mosi 
       ),
       MODE, 8.MHz(), &clocks,
   );
   
   let spiext = SpiExt {
        cs:    gpioa.pa4.into_push_pull_output(), //CsPin         
        busy:  gpiob.pb4.into_floating_input(),   //BusyPin  DI00 
        ready: gpiob.pb5.into_floating_input(),   //ReadyPin DI01 
        reset: gpioa.pa0.into_push_pull_output(), //ResetPin   
        };   


    let mut delay  = dp.TIM5.delay::<1_000_000>(&clocks); 
    let mut delay2 = dp.TIM2.delay::<1_000_000>(&clocks); 

    led.off();
    delay.delay_ms(2000); 
    
    led.on();
    delay.delay_ms(BLINK_DURATION); 
    led.off();

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C

    let mut z = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0);

    let mut disp: Option<DisplayType> = match z.init() {
        Ok(_d)  => {Some(z.into_buffered_graphics_mode())} 
        Err(_e) => {None}
    };

    // let mut screen: [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES] = [R_VAL; DISPLAY_LINES];

    show_message(MODULE_CODE, &mut disp);

    delay.delay_ms(2000); // treated as ms
    //hprintln!("display initialized.").unwrap();


   
    /////////////////////  xca   multiple devices on i2c bus

    let switch1 = Xca9548a::new(i2c1, XcaSlaveAddr::default());

//    let slave_address = 0b010_0000; // example slave address
//    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent
//
//    // Enable channel 0
//    switch1.select_channels(0b0000_0001).unwrap();
//
//    // write to device connected to channel 0 using the I2C switch
//    if switch1.write(slave_address, &write_data).is_err() {
//        //hprintln!("Error write channel 0!").unwrap();
//    }
//
//    // read from device connected to channel 0 using the I2C switch
//    let mut read_data = [0; 2];
//    if switch1.read(slave_address, &mut read_data).is_err() {
//        //hprintln!("Error read channel 0!").unwrap();
//    }
//
//    // write_read from device connected to channel 0 using the I2C switch
//    if switch1
//        .write_read(slave_address, &write_data, &mut read_data)
//        .is_err()
//    {
//        //hprintln!("Error write_read!").unwrap();
//    }


   //   show_message(&"AHT20s on xca", &mut display);


   /////////////////////  sensors on xca    // Start the sensors.
   //  Option allows for the possibility that some sensors are missing.
    hprintln!("xca ");

   //type I2c1Type = I2c<I2C1, Error = ErrorType>;
   type I2c1Type = I2cType<I2C1>;

   type SensType<'a> = AHT20Initialized<'a, I2cSlave<'a, Xca9548a<I2c1Type>, I2c1Type>>;

   //const gives this `static lifetime which might be good for rtic but complicates things here
   //const SENSER: Option::<SensType> = None; 
   //let mut sensors: [Option<SensType>; 8] = [SENSER; 8];
   
   let mut sensors: [Option<SensType>; 8] = [ None, None, None, None, None, None, None, None, ];

   let mut th: [(f32, f32); 8] = [(-500.0, -500.0); 8];

   // Split the device and pass the virtual I2C devices to AHT20 driver
   let switch1parts = switch1.split();

   hprintln!("sensors ");
   //  a loop for this should be possible, but my attempts cause lifetime problems.
   let mut z = AHT20::new(switch1parts.i2c0,  S_ADDR);
   hprintln!("sensor 0 ");
   sensors[0] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };
   hprintln!("sensor 0 init");

   let mut z = AHT20::new(switch1parts.i2c1,  S_ADDR);
   hprintln!("sensor 1 ");
   sensors[1] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };
   hprintln!("sensor 1 init");

   let mut z = AHT20::new(switch1parts.i2c2,  S_ADDR);
   hprintln!("sensor 2 ");
   sensors[2] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };
   hprintln!("sensor 2 init");

   let mut z = AHT20::new(switch1parts.i2c3,  S_ADDR);
   sensors[3] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };

   let mut z = AHT20::new(switch1parts.i2c4,  S_ADDR);
   sensors[4] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };

   let mut z = AHT20::new(switch1parts.i2c5,  S_ADDR);
   sensors[5] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };

   let mut z = AHT20::new(switch1parts.i2c6,  S_ADDR);
   sensors[6] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };

   let mut z = AHT20::new(switch1parts.i2c7,  S_ADDR);
   sensors[7] = match z.init(&mut delay) { Ok(v) => {Some(v)},  Err(_v) => {None} };


   //    screen[0].clear();
   //    write!(screen[0], "    °C %RH").unwrap();


    /////////////////////   lora
    hprintln!("lora");
    

    // cs is named nss on many radio_sx127x module boards
    let z = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, 
                   &CONFIG_RADIO ); 

    let mut lora =  match z {
        Ok(lr)  => {show_message("lora setup ok", &mut disp);
                    //hprintln!("lora setup completed.").unwrap();
                    lr
                   } 
        Err(e)  => {show_message("lora setup Error", &mut disp);
                    //hprintln!("Error in lora setup. {:?}", e).unwrap();
                    asm::bkpt();
                    panic!("{:?}", e) 
                   }
    };
 

    //delay consumed by lora. It is available in lora BUT treats arg as seconds not ms!!
    lora.delay_ms(1);  // arg is being treated as seconds

    hprintln!("loop");
    
    loop {
      led.on(); 
      lora.delay_ms(BLINK_DURATION);  // arg is being treated as seconds
      led.off();
      hprintln!("read sensors");

      for i in 0..sensors.len() {
         th[i] =  match   &mut sensors[i] {
                                 Some(s) => {let v = s.measure(&mut delay2).unwrap();// uses a DelayNs
                                             (v.temperature, v.humidity)
                                            },  
                                 None    => {(-500.0, -500.0)}, // default to transmit for no sensor 
                                };
      };



//      let mut ln = 1;  // screen line to write. rolls if number of sensors exceed DISPLAY_LINES

      show_display(th, &mut disp);

      let message = form_message(th);
      hprintln!("message {:?}", message);
      send(&mut lora, message, &mut disp);

      //delay.delay_ms(READ_INTERVAL);// treated as ms
      lora.delay_ms(READ_INTERVAL);  // treated as seconds
    }
}
