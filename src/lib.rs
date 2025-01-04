
#![no_std]

#![feature(type_alias_impl_trait)]   //  for impl Trait` in type aliases is unstable

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub mod lora;  // has structures and constants

pub mod led;  // LED structures and methods

pub mod setup;

//#[cfg(feature = "stm32f1xx")]
//pub mod  setup_stm32f1xx;

//#[cfg(feature = "stm32f4xx")]   requires feature in Cargo.toml

pub mod  setup_stm32f4xx;

//#[cfg(feature = "stm32f7xx")]
//pub mod  setup_stm32f7xx;
