use anchor::*;
// use cortex_m::peripheral::DWT;
use defmt::*;

use crate::LED_STATE;
use crate::LedState::Connected;
use crate::State;
// use core::sync::atomic::{AtomicU32, Ordering};
use embassy_time::{Instant, TICK_HZ};

// static TICKS_HIGH: AtomicU32 = AtomicU32::new(0);
// static TICKS_LAST: AtomicU32 = AtomicU32::new(0);

// pub fn now_clock32() -> u32 {
//     let ticks = DWT::cycle_count();
//     ticks
// }

// pub fn maintain_clock() -> u32 {
//     let ticks = DWT::cycle_count();
//     if ticks < TICKS_LAST.load(Ordering::Acquire) {
//         TICKS_HIGH.fetch_add(1, Ordering::Release);
//     }
//     TICKS_LAST.store(ticks, Ordering::Release);
//     ticks
// }

// pub fn now_clock64() -> u64 {
//     let ticks = now_clock32();
//     ticks as u64 + ((TICKS_HIGH.load(Ordering::Acquire) as u64) << 32)
// }

// pub fn clock32_to_64(clock32: u32) -> Instant {
//     let now = now_clock64();
//     let high: u32 = (now >> 32) as u32;
//     Instant::from_ticks((clock32 as u64 + ((high as u64) << 32)) / TICKS_TO_CLOCK)
// }

pub fn clock32_to_ticks(clock32: u32) -> u32 {
    clock32 / TICKS_TO_CLOCK as u32
}

pub static TIMER: systick_timer::Timer =
    systick_timer::Timer::new(crate::CLOCK_FREQ_U64, 16_777_215, crate::CLOCK_FREQ_U64);

#[klipper_constant]
pub const CLOCK_FREQ: u32 = 168_000_000;

pub const CLOCK_FREQ_U64: u64 = CLOCK_FREQ as u64;

pub const TICKS_TO_CLOCK: u64 = CLOCK_FREQ_U64 / TICK_HZ;

// #[klipper_constant]
// pub const CLOCK_FREQ: u32 = 48_000_000;

pub fn now_clock32() -> u32 {
    TIMER.now() as u32
}

pub fn now_clock64() -> u64 {
    TIMER.now()
}

pub fn clock32_to_64(clock32: u32) -> Instant {
    let current_time = now_clock64();
    let diff = (current_time as u32).wrapping_sub(clock32) as u64;
    Instant::from_ticks(if diff & 0x8000_0000 != 0 {
        current_time + 0x1_0000_0000 - diff
    } else {
        current_time - diff
    } / TICKS_TO_CLOCK)
}


// pub fn clock32_to_64(clock32: u32) -> Instant {
//     let high = now_clock64() >> 32;
//     Instant::from_ticks(clock32 as u64 + (high << 32))
// }

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
    LED_STATE.signal(crate::LedState::Error);
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
        move_count: u16 = 512
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
pub const STATS_SUMSQ_BASE: u32 = 256;

#[klipper_constant]
const RECEIVE_WINDOW: u32 = 256;

#[klipper_command]
pub fn config_spi_shutdown(_context: &mut State, _oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

#[klipper_command]
pub fn reset() {
    debug!("RESET");
    cortex_m::peripheral::SCB::sys_reset();
}
