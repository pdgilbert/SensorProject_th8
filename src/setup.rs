
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//   //////////////////////////////////////////////////////////////////////

//#[cfg(feature = "stm32f1xx")]
//pub use  crate::setup_all_stm32f1xx::*;

//#[cfg(feature = "stm32f4xx")]  need feature in Cargo.toml for this

pub use  crate::setup_stm32f4xx::*;

//#[cfg(feature = "stm32g4xx")]
//pub use  crate::setup_all_stm32g4xx::*;

