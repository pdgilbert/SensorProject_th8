STATUS:  bin aht20-driver roughly working. Looking for AHT10 crate.

## Summary

This crate has code for a hardware design as in repository
https://github.com/pdgilbert/SensorProject_th8-f401-pcb.
It measures temperature and humidity from eight sensors on a multiplexer. 
The measurements are transmitted by LoRa and optionally displayed on an SSD1306.

The code here is based on code in https://github.com/pdgilbert/rust-integration-testing/examples/projects.
Relative to that repository, this crate is much simplified by targetting specific hardware
and a specific MCU (stm32f401). 
The intention is that code here should be stable and remain working for the hardware.
The code in repository `rust-integration-testing` is intended for testing new versions of 
crates and hals, and as a result is sometimes broken.

As of January 2025 the preferred sensor crates are still under considertation. 
For that reason there are mulitiple bin options, but not all work. 
The multiplexer crate `xca9548a` uses `embedded-hal v1` and that implies sensor devices crates should also.
Currently I have not found a good choice for an AHT10 driver crate using  `embedded-hal v1`.
Testing is being mostly done with AHT20. While many device crates compile, some have run time errors. 
See https://github.com/pdgilbert/i2c-test for more details. Based on test success there,
and needs/nice properties below, attention is currently focussed on `aht20-driver` crate.

The device crate needs/nice properties:
 - `no_std`. 
 - use `embedded_hal v1` (eh-1) to work with the multiplexer crate `xca9548a` using eh-1. 
 - Sensor intialization should return a `Result` so that it is posible to identify when some 
    sensors are not attached.
 - The sensor should use a `DelayNS`. (Automatic if crate uses eh-1.)
 - Preferably the sensor should borrow rather than consume `delay`. If the `delay` is
   consumed then with multiplexing a large number of delays are used. This can
   be done with `cortex_m::asm::delay` but that adds unnecessary complication.
 - It would be nice to allow for software reset so individual sensors can be reset without a power cycle.
 - Syncronous (blocking) is necessary (at the moment because of rtic difficulties) but
     the feature of getting both sync and async from the same crate is nice.
 - Prefer a release version rather than just a repo version.

For additional notes see 
https://github.com/pdgilbert/rust-integration-testing/examples/misc-i2c-drivers/i2c-sensor-notes.txt

## Status and Issues

Check `Cargo.toml` and the code files to see exactly what is being used

 - `aht10`  Not using eh-1 and does not compile.

 - `aht20-blueluna` Uses eh-1. No release version. Consumes delay. Does not return Result.  
              Not compiling, lifetime problems. Abandoned.

 - `embedded-aht20` Uses eh-1. Consumes delay. Both sync and async.  Compiles. Run stalls at ..

 - `aht20-driver`   Uses eh-1. Borrows delay. Compiles and works. 



## Building

Choose the binary in following

```
MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf --features stm32f401   --bin aht10
MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf --features stm32f401   --bin aht20-driver
MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf --features stm32f401   --bin embedded-aht20
```

MONITOR_ID is optional. If not supplied "Txxx" will be used. 
If the binary is too large then `--release` is needed.

## Loading

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run

```
openocd -f interface/stlink-v2.cfg  -f target/stm32f4x.cfg 
```
Adjust interface for your programming dongle.

In another window do
```
MONITOR_ID="whatever"  cargo  run --no-default-features --target thumbv7em-none-eabihf --features stm32f401 --bin aht20-driver
```



## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

## Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
