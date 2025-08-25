use tmc4671;

use crate::{State, TMC_CMD, TMC_RESP};
use anchor::*;
use embassy_futures::block_on;

#[klipper_command]
pub fn spi_transfer(_context: &mut State, oid: u8, data: &[u8]) {
    if let Ok(arr) = data.try_into() {
        block_on(
            TMC_CMD
                .dyn_sender()
                .send(tmc4671::TMCCommand::SpiTransfer(arr)),
        );
        let tmc4671::TMCCommandResponse::SpiResponse(r) = block_on(
            TMC_RESP
                .dyn_subscriber()
                .expect("Should not happen")
                .next_message_pure(),
        );
        klipper_reply!(spi_transfer_response, oid: u8 = oid, response: &[u8] = &r);
    }
}

#[klipper_command]
pub fn spi_send(_context: &mut State, _oid: u8, data: &[u8]) {
    if let Ok(arr) = data.try_into() {
        block_on(
            TMC_CMD
                .dyn_sender()
                .send(tmc4671::TMCCommand::SpiTransfer(arr)),
        );
    }
}

#[klipper_command]
pub fn config_spi(_context: &mut State, _oid: u8, _pin: u32, _cs_active_high: u8) {}

#[klipper_command]
pub fn config_spi_without_cs(_context: &mut State, _oid: u8) {}

#[klipper_command]
pub fn spi_set_bus(_context: &mut State, _oid: u8, _spi_bus: u32, _mode: u32, _rate: u32) {}
