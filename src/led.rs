//! Note that (on board) led pin settings are specific to a board used for testing, despite the cfg feature flags suggesting it may be for a HAL.

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal;

pub trait LED: OutputPin {  // see The Rust Programming Language, section 19, Using Supertraits...
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

    // Note these default methods use delay so DO NOT USE IN rtic.

    fn blink(&mut self, time: u16, delay: &mut impl DelayNs) -> () {
        self.on();
        delay.delay_ms(time.into());
        self.off();
        delay.delay_ms(time.into()); //off for time it was on. consider delay.delay_ms(500);
    }
}
