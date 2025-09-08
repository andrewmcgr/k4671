# A driver and board firmware project for the TMC 4671 FOC motor driver and Kalico/Klipper.

Currently, runs on OpenFFBoard.

Intended to support Ouroboros by Isik's Tech ASAP.

Intended also to serve as a general TMC 4671 driver for Rust projects.

Status:
* starts up the board controller
* detects a 4671
* connects to Kalico via USB-Serial
* establishes clock sync
* SPI passthrough from Kalico allows the Kalico module to also detect the 4671
* Kalico module chip initialisation fails

Useful resources:
* [Kalico/Klipper extras module for TMC 4671](https://github.com/andrewmcgr/tmc-4671) Use this for now if you actually want to use a board in a printer.
* [TMC4671-LA Datasheet v2.08 for chip version 1.3](https://www.analog.com/media/en/technical-documentation/data-sheets/TMC4671-LA_datasheet_rev2.08.pdf) Note: Many obsolete datasheets elsewhere online.
* [Anchor Klipper protocol crate](https://github.com/Annex-Engineering/anchor)
* [Embedded Devices crate](https://crates.io/crates/embedded-devices)
* [Kalico developer docs](https://docs.kalico.gg/Code_Overview.html)
* [Embassy](https://embassy.dev)
* [OpenFFBoard hardware repository (schematics etc)](https://github.com/Ultrawipf/OpenFFBoard-hardware)

Also contains stepper protocol code derived from Flycron.
