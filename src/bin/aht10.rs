//! Measure temperature and humidity on eight AHT10 sensors on I2Cx, multiplexed with crate 
//! xcs9548a because all sensors use the same addrss.
//! Display using SSD1306 on I2Cx.
//! Transmit with LoRa.

//!  To Do:
//!    - make work.

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

const MODULE_CODE:  &str = "th8-f401"; 
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
//use cortex_m_semihosting::{hprintln};
use cortex_m::asm; // for delay
use cortex_m_rt::entry;

use core::fmt::Write;

use stm32f4xx_hal::{
    pac::{Peripherals, I2C1, I2C2, I2C3, SPI1, TIM2, TIM5},
    timer::{Delay as halDelay},
    rcc::{RccExt},
    spi::{Spi},
    i2c::I2c as I2cType,   //this is a type vs  embedded_hal::i2c::I2c trait
    gpio::{Output, PushPull, GpioExt, Input},
    prelude::*,
    block,
    timer::{TimerExt},
    gpio::{Pin}, 
    gpio::{gpioa::{PA0, PA1, PA8, PA4, }},
    gpio::{gpiob::{PB4, PB5, PB6, PB7, PB8}},
    gpio::{gpioc::{PC13}},
};


use embedded_hal::{spi::{Mode, Phase, Polarity},
                   i2c::{I2c, ErrorType},   //trait
                   //delay::DelayNs,
                   digital::OutputPin,
};



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

use xca9548a::{Error::I2C, Error as XcaError, SlaveAddr as XcaSlaveAddr, Xca9548a, I2cSlave}; 


/////////////////////  aht10

use embedded_hal::delay::DelayNs;
use aht10::{AHT10, Humidity, Temperature, Error as aht10Error, }; 
//use embedded_hal_async::delay::DelayNs;
//use aht10_async::{AHT10, Humidity, Temperature, Error as aht10Error, }; 

// A delay is used in sensor initializations. 
// asm::delay is not an accurate timer but gives a delay at least number of indicated clock cycles.
//use embedded_hal_02::blocking::delay::{DelayMs};  //sensor crate needs old delay

use cortex_m::asm::delay; // argment in clock cycles so (5 * CLOCK) cycles gives aprox 5 second delay
pub const  ALTCLOCK: u32 = 16_000_000;

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
pub struct AltDelay {}

impl DelayNs for AltDelay {
    fn delay_ns(&mut self, t:u32) {
        delay((t as u32) * (ALTCLOCK / 1_000_000_000)); 
    }
    fn delay_us(&mut self, t:u32) {
        delay((t as u32) * (ALTCLOCK / 1_000_000)); 
    }
    fn delay_ms(&mut self, ms: u32) {
        delay((ms as u32) * (ALTCLOCK / 1000)); 
    }
}

//impl DelayMs<u16> for AltDelay {
impl embedded_hal_02::blocking::delay::DelayMs<u16> for AltDelay {
    fn delay_ms(&mut self, ms: u16) {
       delay((ms as u32) * (ALTCLOCK / 1000)); 
    }
}
//   //////////////////////////////////////////////////////////////////

    fn show_display(
        th: [(i64, i64); 8],
        //disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        disp: &mut Option<DisplayType>,
    ) -> ()
    {    
     if disp.is_some() { // show_message does nothing if disp is None. 
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines
       let(t,h) = th[0];
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line,   "{}°C  {}",  t,   h).unwrap();
     //  write!(line,   "1:{:3}.{:1}  {:3}.{:1}°C",  t[0]/10,t[0].abs() %10,   t[1]/10,t[1].abs() %10).unwrap();

   // CHECK SIGN IS CORRECT FOR -0.3 C
   // NEED TO DISPLAY THE REST
      // hprintln!(" t {:?} = 10 * degrees", t).unwrap();

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
            th: [(i64, i64); 8],
           ) -> heapless::Vec<u8, MESSAGE_LEN> {

        let mut line: heapless::Vec<u8, MESSAGE_LEN> = heapless::Vec::new(); 
        let mut temp: heapless::Vec<u8, S_FMT> = heapless::Vec::new(); 

        // Consider handling error in next. If line is too short then attempt to write it crashes
        
        for i in 0..MONITOR_IDU.len() { line.push(MONITOR_IDU[i]).unwrap()};
        line.push(b'<').unwrap();
        
        for i in 0..th.len() {
                //temp.clear();
                //hprintln!(" J{}:{:3}.{:1}",      i+1, t[i]/10, t[i].abs() %10).unwrap(); // t[0] is for J1
                //write!(temp,  " J{}:{:3}.{:1}",  i+1, t[i]/10, t[i].abs() %10).unwrap(); // must not exceed S_FMT
                //hprintln!("temp {:?}  temp.len {}", temp, temp.len()).unwrap();
                //for j in 0..temp.len() {line.push(temp[j]).unwrap()};
                //for j in 0..th.len() {line.push(th[i].0).unwrap()};
                for j in 0..th.len() {line.push(0).unwrap()};   //FAKE
        };

        line.push(b'>').unwrap();
         
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

   // this compiles, but pc9 is not available on blackpill f401. Syntax pc9..._drain::<4>() also works
   let sda = gpioc.pc9.into_alternate_open_drain();
   let scl = gpioa.pa8.into_alternate_open_drain();
   let i2c3 = I2cType::new(dp.I2C3, (scl, sda), 400.kHz(), &clocks); 

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


   let mut delay = dp.TIM5.delay::<8_000_000>(&clocks);  //CHECK

    led.off();
    delay.delay_ms(2000); // treated as ms
    
    led.on();
    delay.delay_ms(BLINK_DURATION); // treated as ms
    led.off();

    /////////////////////   ssd

    let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C

    let mut z = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0);

    let mut disp: Option<DisplayType> = match z.init() {
        Ok(_d)  => {Some(z.into_buffered_graphics_mode())} 
        Err(_e) => {None}
    };

//    let mut screen: [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES] = [R_VAL; DISPLAY_LINES];

    show_message(MODULE_CODE, &mut disp);

    delay.delay_ms(2000); // treated as ms
    //hprintln!("display initialized.").unwrap();


   
    /////////////////////  xca   multiple devices on i2c bus
//  CHECK https://github.com/rahix/port-expander

    let slave_address = 0b010_0000; // example slave address
    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let mut switch1 = Xca9548a::new(i2c1, XcaSlaveAddr::default());

    // Enable channel 0
    switch1.select_channels(0b0000_0001).unwrap();

    // write to device connected to channel 0 using the I2C switch
    if switch1.write(slave_address, &write_data).is_err() {
        //hprintln!("Error write channel 0!").unwrap();
    }

    // read from device connected to channel 0 using the I2C switch
    let mut read_data = [0; 2];
    if switch1.read(slave_address, &mut read_data).is_err() {
        //hprintln!("Error read channel 0!").unwrap();
    }

    // write_read from device connected to channel 0 using the I2C switch
    if switch1
        .write_read(slave_address, &write_data, &mut read_data)
        .is_err()
    {
        //hprintln!("Error write_read!").unwrap();
    }

 //   show_message(&"AHT10s on xca", &mut display);

    /////////////////////  AHT10s on xca    // Start the sensors.
    

//   //type I2c1Type = I2c<I2C1, Error = ErrorType>;
//   type I2c1Type = I2cType<I2C1>;
//   
//   type SensType<'a> = AHT10<I2cSlave<'a,  Xca9548a<I2c1Type>, I2c1Type>, AltDelay>;
//   
//       const SENSER: Option::<SensType> = None;      //const gives this `static lifetime
//       let mut sensors: [Option<SensType>; 16] = [SENSER; 16];

//use shared_bus::{I2cProxy};

    // Split the device and pass the virtual I2C devices to AHT10 driver
    let switch1parts = switch1.split();

 //    let parts  = [switch1parts.i2c0, switch1parts.i2c1, switch1parts.i2c2, switch1parts.i2c3,
 //                  switch1parts.i2c4, switch1parts.i2c5, switch1parts.i2c6, switch1parts.i2c7];


////////////////////////////////////// compiles  
    let mut some_driver = Driver::new(switch1parts.i2c1);
    let mut some_other_driver = Driver::new(switch1parts.i2c2);
    some_driver.do_something().unwrap();
    some_other_driver.do_something().unwrap();

/// Some driver defined in a different crate.
/// Defined here for completeness.
struct Driver<I2C> {
    i2c: I2C,
}

impl<I2C, E> Driver<I2C>
where
    I2C: I2c<Error = E>,
    E: core::fmt::Debug,
{
    pub fn new(i2c: I2C) -> Self {
        Driver { i2c }
    }
    pub fn do_something(&mut self) -> Result<(), XcaError<E>> {
        self.i2c.write(0x21, &[0x01, 0x02]).map_err(XcaError::I2C)
    }
}


////////////////////////////////////// fails   LOOK FOR AHT10 USING EH 1
//let mut z = AHT10::new( switch1parts.i2c3, AltDelay{}) ; //
//let mut z = AHT10::new( i2c3, delay) ; //
//let mut z = AHT10::new(i2c3, AltDelay{}) ; // compiles 
 
struct Sens<I2C> {
    i2c: I2C,
}

impl<I2C, E> Sens<I2C>   //wrapper                                        //
where                                                                     //
    I2C: I2c<Error = E>,                                                  //
    E: core::fmt::Debug,                                                  //
{                                                                         //
    pub fn new(i2c: I2C) -> Self {                                        //
        //let z = AHT10::new( i2c, AltDelay{}) ;                                     //
        Sens { i2c }
    }                                                                     //
    pub fn do_something(&mut self) -> Result<(), XcaError<E>> {           //
        self.i2c.write(0x21, &[0x01, 0x02]).map_err(XcaError::I2C)        //
    }                                                                     //
}                                                                         //
                                                                          //
let mut z = Sens::new(switch1parts.i2c1);                                 //
let z = AHT10::new(switch1parts.i2c3, AltDelay{});

////////////////////////////////////// 


    let mut i = 0;  // not very elegant
//    for  prt in parts {

//  struct MySens {
//      sensor : i32,
//  }
//  
//  let z : impl Read = ();
//  
//  impl I2c for MySens {
//     fn transaction(&self, address, operation) -> i32 {
//        42
//     }
//   }

//  impl Read for MySens {
//     fn read(&self) -> i32 {
//        42
//     }
//   }
//  
//  impl Write for MySens {
//     fn write(&self, x:i32) -> () {
//        self.sensor = x
//        ()
//     }
//   }

//  impl ReadReady for MySens {
//     fn read(&self) -> i32 {
//        42
//     }
//   }
//  
//  impl WriteReady for MySens {
//     fn read(&self) -> i32 {
//        42
//     }
//   }


//   
//         let z = AHT10::new(prt, AltDelay{});
//  
//         match z {
//             Ok(mut v) => {v.reset().expect("sensor reset failed");  //should handle this 
//                           sensors[i] = Some(v);
//                           //sensors[i] = v;
//                         },
//             Err(_e)   => {//hprintln!("J{} unused", i).unwrap();
//                           show_message("J{} unused",  &mut disp);
//                          },
//         }
//         delay.delay_ms(500);
//         
//         i += 1;
//         //hprintln!("i+1 {}", i).unwrap();
//      };

//    screen[0].clear();
//    write!(screen[0], "    °C %RH").unwrap();


    /////////////////////   lora

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


    /////////////////////   loop
    //hprintln!("entering loop").unwrap();
    
    loop { }
}
