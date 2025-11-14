use anchor::*;
use defmt::*;

use crate::{State, leds::{LED_STATE, LedState}};
use embassy_time::{Instant, TICK_HZ};

pub fn clock32_to_ticks(clock32: u32) -> u32 {
    clock32 / CLOCKS_PER_TICK as u32
}

pub fn clocki16_to_ticks(add: i16) -> i16 {
    add as i16 / CLOCKS_PER_TICK as i16
}

pub static TIMER: systick_timer::Timer =
    systick_timer::Timer::new(crate::CLOCK_FREQ_U64, 16_777_215, crate::CLOCK_FREQ_U64);

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 168_000_000;

pub const CLOCK_FREQ_U64: u64 = CLOCK_FREQ as u64;

pub const CLOCKS_PER_TICK: u64 = CLOCK_FREQ_U64 / TICK_HZ;

pub fn now_clock32() -> u32 {
    TIMER.now() as u32
}

pub fn now_clock64() -> u64 {
    TIMER.now()
}

pub fn clock32_to_clock64(clock32: u32) -> u64 {
    let current_time = now_clock64();
    let diff = (current_time as u32).wrapping_sub(clock32) as u64;
    if diff & 0x8000_0000 != 0 {
        current_time + 0x1_0000_0000 - diff
    } else {
        current_time - diff
    }
}

pub fn clock32_to_instant(clock32: u32) -> Instant {
    Instant::from_ticks(clock32_to_clock64(clock32) / CLOCKS_PER_TICK)
}

pub fn duration_to_ticks(dur: embassy_time::Duration) -> u32 {
    (dur.as_ticks() as u64 * TICK_HZ / CLOCK_FREQ_U64) as u32
}

pub fn instant_to_clock32(instant: Instant) -> u32 {
    let ticks = instant.as_ticks() as u64;
    let clocks = ticks * CLOCKS_PER_TICK;
    (clocks & 0xFFFF_FFFF) as u32
}

#[klipper_constant]
#[expect(non_upper_case_globals)]
const BUS_PINS_spi1: &str = "spi1_miso,spi1_clk,spi1_mosi";

#[klipper_command]
pub fn get_uptime() {
    let c = now_clock64();
    let clock: u32 = c as u32;
    let high: u32 = (c >> 32) as u32;
    klipper_reply!(
        uptime,
        high: u32,
        clock: u32
    );
    debug!("uptime {} {} {}", c, high, clock);
}

#[klipper_command]
pub fn get_clock() {
    let clock = now_clock32();
    klipper_reply!(clock, clock: u32);
    debug!("clock {}", clock);
}

#[klipper_command]
pub fn emergency_stop(context: &mut State) {
    debug!("EMERGENCY STOP");
    LED_STATE.signal(LedState::Error);
    for i in 0..context.steppers.len() {
        context.steppers[i].stop();
    }
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
        move_count: u16 = 64
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
    LED_STATE.signal(LedState::Connected);
    context.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {
    debug!("Alloc oids {}", _count);
}

#[klipper_constant]
const MCU: &str = "k4671_openffboard";

#[klipper_constant]
pub const STATS_SUMSQ_BASE: u32 = 256;

#[klipper_constant]
const RECEIVE_WINDOW: u32 = 16;

#[klipper_command]
pub fn config_spi_shutdown(_context: &mut State, _oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

#[klipper_command]
pub fn reset() {
    debug!("RESET");
    cortex_m::peripheral::SCB::sys_reset();
}
