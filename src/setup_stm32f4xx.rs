pub use stm32f4xx_hal::pac::{Peripherals};

use stm32f4xx_hal::{
      pac::{I2C1, I2C2, SPI1, TIM5 },
      timer::{Delay as timDelay, TimerExt},
      rcc::{RccExt},
      spi::{Spi},
      i2c::I2c as I2cType,   //this is a type
      gpio::{Output, PushPull, GpioExt, Input,
             gpioa::{PA0, PA4, },
             gpiob::{PB4, PB5, },
             gpioc::{PC13 as LEDPIN}
      },
      prelude::*,
};

use embedded_hal::delay::DelayNs;

// for spi::Mode; // possibly from  lora instead
use embedded_hal::{spi,};    //for ::{Mode, Phase, Polarity}, 

//                   digital::OutputPin,};                   

const TIM_FREQ: u32 =  1000000;
pub type Delay1Type = timDelay<TIM5, TIM_FREQ>;  // lora uses this too
pub type Delay2Type = impl DelayNs; // or timDelay<TIM2, TIM_FREQ>;


type I2c1Type = I2cType<I2C1>;
type I2c2Type = I2cType<I2C2>;

pub use crate::led::LED;  // defines trait and default methods
type LedType = LEDPIN<Output<PushPull>>;
impl LED for LedType {}    

/////////////////////////////  spi  //////////////////////////////////

type Cs    = PA4<Output<PushPull>>;
type Busy  = PB4<Input<>>;
type Ready = PB5<Input<>>;
type Reset = PA0<Output<PushPull>>;

pub struct SpiExt { pub cs:    Cs, 
                    pub busy:  Busy, 
                    pub ready: Ready, 
                    pub reset: Reset
}

//  SPI mode for radio
pub const MODE: spi::Mode = spi::Mode {
    phase:  spi::Phase::CaptureOnSecondTransition,
    polarity:  spi::Polarity::IdleHigh,
};


/////////////////////////////   setup  //////////////////////////////////

pub fn setup_from_dp(dp: Peripherals) -> (
             I2c1Type, I2c2Type, 
             Spi<SPI1>, SpiExt, 
             LedType, 
             Delay1Type, Delay2Type) {

   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();
  
   let gpioa = dp.GPIOA.split();
   let gpiob = dp.GPIOB.split();
   let gpioc   = dp.GPIOC.split();

   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let sda = gpiob.pb9.into_alternate_open_drain(); 
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


   let mut delay1 = dp.TIM5.delay::<TIM_FREQ>(&clocks);
   let delay2 = dp.TIM2.delay::<TIM_FREQ>(&clocks); 

    led.off();
    delay1.delay_ms(1000); 
    
    led.blink(200, &mut delay1); 

   (i2c1, i2c2, spi, spiext, led, delay1, delay2)
}




