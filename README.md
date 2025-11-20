# A driver and board firmware project for the TMC 4671 FOC motor driver and Kalico/Klipper.

Currently, runs on OpenFFBoard.

Intended to support Ouroboros by Isik's Tech ASAP. Board abstractions are not yet complete.

Intended also to serve as a general TMC 4671 driver for Rust projects.

Status:
* starts up the board controller
* detects a 4671
* connects to Kalico via USB-Serial
* Clock sync. Note that the clock rate reported to Klipper is much lower than the actual CPU clock of the board.
* Stats (CPU usage on MCU)
* Trigger Sync (multi-MCU homing; note this does not include endstops on this MCU or sensorless homing)
* Feedforward current control
* Not yet reliable, but can home, QGL, and start printing

Build and run:

Can be run with `DEFMT_LOG=debug cargo run --release | tee ~/trace` if a Probe-RS compatible probe is available.

Can be built with `DEFMT_LOG=off cargo build --release`, converted to a .bin with `llvm-objcopy -O binary target/thumbv7em-none-eabi/release/k4671 k4671.bin` and loaded on the board with DFU.

Klipper configuration (substitute your own serial number):
```
[mcu tmcx]
serial: /dev/serial/by-id/usb-k4671_K4671_Motor_Driver_2F0028000850324643303520-if00

[stepper_x]
step_pin: tmcx:step
dir_pin: !tmcx:dir
enable_pin: tmcx:enable
microsteps: 2
full_steps_per_rotation: 4096
rotation_distance: 40
```
Yes, really, that is the entire config.

See `tmc4671/src/config.rs` for the driver configuration. There will be more convenient means to adjust this later in development.

Useful resources:
* [Kalico/Klipper extras module for TMC 4671](https://github.com/andrewmcgr/tmc-4671) Use this for now if you actually want to use a board in a printer.
* [TMC4671-LA Datasheet v2.08 for chip version 1.3](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC4671-LA_datasheet_rev2.08.pdf) Note: Many obsolete datasheets elsewhere online.
* [Anchor Klipper protocol crate](https://github.com/Annex-Engineering/anchor)
* [Embedded Devices crate](https://crates.io/crates/embedded-devices)
* [Kalico developer docs](https://docs.kalico.gg/Code_Overview.html)
* [Embassy](https://embassy.dev)
* [OpenFFBoard hardware repository (schematics etc)](https://github.com/Ultrawipf/OpenFFBoard-hardware)

Also contains stepper protocol code derived from Flycron.
