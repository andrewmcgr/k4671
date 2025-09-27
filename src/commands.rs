use anchor::*;
use defmt::*;

use crate::LED_STATE;
use crate::LedState::Connected;
use crate::State;
use embassy_time::Instant;

#[klipper_constant]
#[expect(non_upper_case_globals)]
const BUS_PINS_spi1: &str = "spi1_miso,spi1_clk,spi1_mosi";

#[klipper_constant]
const CLOCK_FREQ: u32 = 84_000_000;

#[klipper_command]
pub fn get_uptime(_context: &mut crate::State) {
    let c = Instant::now().as_ticks();
    let clock: u32 = (c & 0xFFFF_FFFF) as u32;
    let high: u32 = (c >> 32) as u32;
    debug!("uptime {} {} {}", c, high, clock);
    klipper_reply!(
        uptime,
        high: u32,
        clock: u32
    );
}

#[klipper_command]
pub fn get_clock() {
    let c = Instant::now().as_ticks();
    debug!("clock {}", c);
    klipper_reply!(clock, clock: u32 = (c & 0xFFFF_FFFF) as u32);
}

#[klipper_command]
pub fn emergency_stop() {
    debug!("EMERGENCY STOP");
    LED_STATE.signal(crate::LedState::Error);
}

#[klipper_command]
pub fn get_config(context: &State) {
    let crc = context.config_crc;
    debug!("get_config {:x}", crc);
    klipper_reply!(
        config,
        is_config: bool = crc.is_some(),
        crc: u32 = crc.unwrap_or(0),
        is_shutdown: bool = false,
        move_count: u16 = 2000
    );
}

#[klipper_command]
pub fn config_reset(context: &mut State) {
    debug!("config_reset");
    context.config_crc = None;
}

#[klipper_command]
pub fn finalize_config(context: &mut State, crc: u32) {
    debug!("finalize_config {:x}", crc);
    LED_STATE.signal(Connected);
    context.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {
    debug!("Alloc oids {}", _count);
}

#[klipper_constant]
const MCU: &str = "k4671_openffboard";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;

#[klipper_command]
pub fn config_spi_shutdown(_context: &mut State, _oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

#[klipper_command]
pub fn reset() {
    debug!("RESET");
    cortex_m::peripheral::SCB::sys_reset();
}
