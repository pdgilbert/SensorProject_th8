STATUS:  Draft

##  Contents

See the auto-generated menu in the github README display (above right).

## Summary

This crate has code for a hardware design as in repository
https://github.com/pdgilbert/SensorProject_th8-f401-pcb.
It measures temperature  and humidity from eight AHT10s or AHT20s. 
The measurements are transmitted by LoRa and optionally displayed on an SSD1306.

The code here is based on code in https://github.com/pdgilbert/rust-integration-testing/examples/projects.
Relative to that repository, this crate is much simplified by targetting specific hardware
and a specific MCU (stm32f401). 
The intention is that code here should be stable and remain working for the hardware.
The code in repository `rust-integration-testing` is intended for testing new versions of 
crates and hals, and as a result is sometimes broken.

## Building

```
MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf --features stm32f401   --bin aht10   [ --release ]

MONITOR_ID="whatever"  cargo build --no-default-features  --target thumbv7em-none-eabihf --features stm32f401   --bin aht20   [ --release ]
```

MONITOR_ID is optional. If not supplied "Txxx" will be used. 


## Loading

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run

```
openocd -f interface/stlink-v2.cfg  --target target/stm32f4x.cfg 
```
Adjust interface for your programming dongle.

In another window do
```
MONITOR_ID="whatever"  cargo  run --no-default-features --target thumbv7em-none-eabihf --features stm32f401 --bin aht20   [ --release]
```
The `--release` will be needed if code is too big for memory.
cargo  run --target $TARGET --features $HAL,$MCU  [ --release]

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
