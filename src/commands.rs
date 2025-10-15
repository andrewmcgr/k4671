use anchor::*;
use defmt::*;

use crate::LED_STATE;
use crate::LedState::Connected;
use crate::State;
use embassy_time::{Instant, TICK_HZ};


#[klipper_constant]
const CLOCK_FREQ: u32 = 24_000_000;

pub const CLOCKS_PER_TICK: u32 = CLOCK_FREQ / TICK_HZ as u32;

fn now() -> Instant {
    Instant::now()
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct Clock64(u64);

impl Clock64 {
    pub fn as_ticks(&self) -> u64 {
        self.0 / CLOCKS_PER_TICK as u64
    }
    pub fn now() -> Self {
        Clock64::from(now())
    }
}

impl From<Clock64> for u64 {
    fn from(clock: Clock64) -> Self {
        clock.0
    }
}
impl From<Clock64> for Instant {
    fn from(clock: Clock64) -> Instant {
        Instant::from_ticks(clock.0 / CLOCKS_PER_TICK as u64)
    }
}
impl From<Instant> for Clock64 {
    fn from(i: Instant) -> Self {
        Clock64(i.as_ticks() * CLOCKS_PER_TICK as u64)
    }
}
impl From<u32> for Clock64 {
    fn from(val: u32) -> Self {
        Clock64::from(Clock32(val))
    }
}
impl From<Clock32> for Clock64 {
    fn from(clock32: Clock32) -> Clock64 {
        let now_ticks = Instant::now().as_ticks();
        let diff = (now_ticks as u32).wrapping_sub(clock32.0) as u32;
        if diff & 0x8000_0000 != 0 {
            Clock64(now_ticks + 0x1_0000_0000 - diff as u64)
        } else {
            Clock64(now_ticks - diff as u64)
        }
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct Clock32(u32);

impl Clock32 {
    pub fn now() -> Self {
        Clock32::from(now())
    }
}

impl From<Clock32> for u32 {
    fn from(clock32: Clock32) -> Self {
        clock32.0
    }
}
impl From<Clock64> for Clock32 {
    fn from(clock64: Clock64) -> Self {
        Clock32(clock64.0 as u32)
    }
}
impl From<u32> for Clock32 {
    fn from(val: u32) -> Self {
        Clock32(val)
    }
}
impl From<Instant> for Clock32 {
    fn from(i: Instant) -> Self {
        Clock32((i.as_ticks() * CLOCKS_PER_TICK as u64) as u32)
    }
}

#[klipper_constant]
#[expect(non_upper_case_globals)]
const BUS_PINS_spi1: &str = "spi1_miso,spi1_clk,spi1_mosi";


#[klipper_command]
pub fn get_uptime() {
    let c = u64::from(Clock64::now());
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
    let c = u32::from(Clock32::now());
    klipper_reply!(clock, clock: u32 = c);
    debug!("clock {}", c);
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
        move_count: u16 = 113
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

// #[klipper_constant]
// const RECEIVE_WINDOW: u32 = 512;

#[klipper_command]
pub fn config_spi_shutdown(_context: &mut State, _oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

#[klipper_command]
pub fn reset() {
    debug!("RESET");
    cortex_m::peripheral::SCB::sys_reset();
}
