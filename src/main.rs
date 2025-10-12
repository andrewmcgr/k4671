#![no_std]
#![no_main]

use cortex_m::peripheral::DWT;
use cortex_m_rt::entry;

use assign_resources::assign_resources;
use core::cell::RefCell;
use core::ops::DerefMut;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{Executor, InterruptExecutor};
pub use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{Config, Peri, bind_interrupts, peripherals, spi, usb};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, TICK_HZ, Timer};
use static_cell::StaticCell;

use heapless::{LinearMap, Vec};

use anchor::*;
use tmc4671::{self, CS, TimeIterator, config::TMC4671Config};
use {defmt_rtt as _, panic_probe as _};
mod commands;
mod leds;
mod stepper;
mod stepper_commands;
mod target_queue;
mod usb_anchor;
use crate::leds::blink;

pub type EmulatedStepper = stepper::EmulatedStepper<tmc4671::TMCTimeIterator, 512>;
pub type ProtectedEmulatedStepper = CriticalSectionMutex<RefCell<EmulatedStepper>>;
pub type ProtectedTrSync = CriticalSectionMutex<RefCell<TrSync>>;

const NUM_STEPPERS: usize = 1;
static STEPPER_POOL: StaticCell<[ProtectedEmulatedStepper; NUM_STEPPERS]> = StaticCell::new();
static STEPPERS: StaticCell<&'static [ProtectedEmulatedStepper; NUM_STEPPERS]> = StaticCell::new();

const NUM_TRSYNC: usize = 8;
static TRSYNC_POOL: StaticCell<[ProtectedTrSync; NUM_TRSYNC]> = StaticCell::new();
static TRSYNC: StaticCell<&'static [ProtectedTrSync; NUM_TRSYNC]> = StaticCell::new();
pub static TRSYNC_WATCH: Watch<CriticalSectionRawMutex, usize, 2> = Watch::new();

klipper_config_generate!(
  transport = crate::TRANSPORT_OUTPUT: crate::BufferTransportOutput,
  context = &'ctx mut crate::State,
);

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

assign_resources! {
    led: LedResources {
        led: PD7,
        focled: PE0,
        errled: PE1,
    }
    tmc: TmcResources {
        miso: PA6,
        mosi: PA7,
        clk: PA5,
        cs: PA4,
        spi: SPI1,
        dma_a: DMA2_CH3,
        dma_b: DMA2_CH0,
        enable: PE7,
        flag: PE8,
        brake: PE10,
    }
    usb: UsbResources {
        otg: USB_OTG_FS,
        dplus: PA12,
        dminus: PA11,
    }
    encoder: EncoderResources {
        enc_a: PC6,
        enc_b: PC7,
        enc_z: PD15,
        timer: TIM3,
    }
}

klipper_enumeration!(
    #[expect(non_camel_case_types)]
    enum spi_bus {
        spi1,
    }
);

klipper_enumeration!(
    #[expect(non_camel_case_types)]
    enum pin {
        spi1_cs,
    }
);

pub fn clock32_to_64(clock32: u32) -> Instant {
    let now_ticks = Instant::now().as_ticks();
    let diff = (now_ticks as u32).wrapping_sub(clock32) as u32;
    Instant::from_ticks(if diff & 0x8000_0000 != 0 {
        now_ticks + 0x1_0000_0000 - diff as u64
    } else {
        now_ticks - diff as u64
    })
}

#[derive(Debug, Default)]
pub struct TrSync {
    oid: Option<u8>,
    report_clock: Option<Instant>,
    report_ticks: Option<u32>,
    expire_reason: u8,
    trigger_reason: u8,
    timeout_clock: Option<Instant>,
    stepper_oids: Vec<u8, NUM_STEPPERS>,
    can_trigger: bool,
}

impl TrSync {
    pub const fn new() -> Self {
        Self {
            oid: None,
            report_clock: None,
            report_ticks: None,
            expire_reason: 0,
            trigger_reason: 0,
            timeout_clock: None,
            can_trigger: false,
            stepper_oids: heapless::Vec::new(),
        }
    }

    fn process_trsync(self: &mut TrSync, oid: u8) -> Option<Instant> {
        // trsync processing
        let now = Instant::now();
        let mut rv = Instant::MAX;
        if let Some(report_clock) = self.report_clock {
            info!(
                "TrSync check {} now {} report {}",
                oid,
                now.as_ticks(),
                report_clock.as_ticks()
            );
            if now >= report_clock {
                // Timer has expired
                if let Some(ticks) = self.report_ticks {
                    info!(
                        "TrSync report {} now {} ticks {}",
                        oid,
                        now.as_ticks(),
                        ticks
                    );
                    let next = report_clock + Duration::from_ticks(ticks as u64);
                    self.report_clock = Some(next);
                    if next < rv {
                        rv = next;
                    }
                } else {
                    info!("TrSync report {} now {} ticks None", oid, now.as_ticks());
                    self.report_clock = None;
                }
                stepper_commands::trsync_report(
                    oid,
                    if self.can_trigger { 1 } else { 0 },
                    self.trigger_reason,
                    now.as_ticks() as u32,
                );
            } else {
                if let Some(next) = self.report_clock {
                    if next < rv {
                        rv = next;
                    }
                }
            }
        }
        if self.can_trigger && self.timeout_clock.is_some() {
            info!(
                "TrSync check {} now {} timeout {}",
                oid,
                now.as_ticks(),
                self.timeout_clock.unwrap().as_ticks()
            );
            if now > self.timeout_clock.unwrap() {
                info!("TrSync timeout {} now {}", oid, now.as_ticks());
                // Timer has expired
                self.timeout_clock = None;
                self.can_trigger = false;
                self.trigger_reason = self.expire_reason;
                stepper_commands::trsync_report(oid, 0, self.expire_reason, 0);
                // rv = None;
            } else {
                if let Some(next) = self.timeout_clock {
                    if next < rv {
                        rv = next;
                    }
                }
            }
        }
        info!("TrSync done {} returns {}", oid, rv);
        return if rv == Instant::MAX { None } else { Some(rv) };
    }
}

pub struct State {
    config_crc: Option<u32>,
    steppers: &'static [ProtectedEmulatedStepper; NUM_STEPPERS],
    steppers_by_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    steppers_by_enable_oid: LinearMap<u8, usize, NUM_STEPPERS>,
    trsync: &'static [ProtectedTrSync; NUM_TRSYNC],
}

impl State {
    pub fn new(
        steppers: &'static [ProtectedEmulatedStepper; NUM_STEPPERS],
        trsync: &'static [ProtectedTrSync; NUM_TRSYNC],
    ) -> Self {
        Self {
            config_crc: None,
            steppers,
            steppers_by_oid: LinearMap::new(),
            steppers_by_enable_oid: LinearMap::new(),
            trsync,
        }
    }
}

pub static USB_OUT_BUFFER: usb_anchor::AnchorPipe = usb_anchor::AnchorPipe::new();

pub(crate) struct BufferTransportOutput;

impl TransportOutput for BufferTransportOutput {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        f(&mut scratch);
        let output = scratch.result();
        if let Ok(n) = USB_OUT_BUFFER.try_write(output) {
            if n < output.len() {
                // Retry, possible a ring buffer wrap
                debug!("USB transmit buffer retry???");
                let _ = USB_OUT_BUFFER.try_write(&output[n..]);
            }
        } else {
            debug!("USB transmit buffer full???");
        }
    }
}

#[embassy_executor::task]
async fn trsync_processing(trsync: &'static [ProtectedTrSync; NUM_TRSYNC]) {
    let mut receiver = TRSYNC_WATCH.receiver().unwrap();
    let sender = TRSYNC_WATCH.sender();

    loop {
        // Wait for something to do
        receiver.changed_and(|v| *v > 0).await;
        info!("TrSync wake");
        loop {
            let mut ticks = Instant::MAX;

            for t in trsync.iter() {
                t.lock(|t| {
                    let mut t = t.borrow_mut();
                    let t = t.deref_mut();
                    trace!("TrSync processing {}", t.oid);
                    if let Some(oid) = t.oid {
                        if let Some(time) = t.process_trsync(oid)
                            && time < ticks
                        {
                            info!("TrSync next candidate {} at {}", oid, time.as_ticks());
                            ticks = time;
                        }
                    }
                });
            }

            if ticks != Instant::MAX {
                info!("TrSync next at {}", ticks.as_ticks());
                Timer::at(ticks).await
            } else {
                // Nothing to do, go back to sleep
                info!("TrSync idle");
                sender.send(0);
                break;
            }
        }
    }
}

pub(crate) const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

pub static TMC_CMD: tmc4671::TMCCommandChannel = tmc4671::TMCCommandChannel::new();
pub static TMC_RESP: tmc4671::TMCResponseBus = tmc4671::TMCResponseBus::new();

fn process_moves(stepper: &mut EmulatedStepper, next_time: Instant) {
    // static mut LAST_POS: i32 = 0;
    let crate::target_queue::ControlOutput {
        position: target_position,
        position_1: c1,
        position_2: c2,
    } = stepper.target_queue.get_for_control(next_time);
    // if target_position != unsafe { LAST_POS } {
    //     trace!("Target pos {} {}", target_position, next_time.as_ticks());
    //     unsafe { LAST_POS = target_position };
    // }

    let c1 = c1.map(|(t, p)| (Instant::from_ticks(t), p));
    let c2 = c2.map(|(t, p)| (Instant::from_ticks(t), p));

    let v0 = match c1 {
        Some((t1, p1)) => {
            (((p1 as i32) - (target_position)) as f32)
                / ((t1 - next_time).as_ticks() as f32 / (TICK_HZ as f32))
        }
        _ => 0.0,
    };
    let v1 = match (c1, c2) {
        (Some((t1, p1)), Some((t2, p2))) => {
            (((p2 as i32) - (p1 as i32)) as f32) / ((t2 - t1).as_ticks() as f32 / (TICK_HZ as f32))
        }
        _ => 0.0,
    };
    let a0 = match (v0, v1, c1, c2) {
        (v0, v1, Some((t1, _)), Some((t2, _))) => {
            (v1 - v0) / ((t2 - t1).as_ticks() as f32 / (TICK_HZ as f32))
        }
        _ => 0.0,
    };
    // debug!("Send move {}", target_position);
    const STEP_MULT: i32 = 8;
    TMC_CMD
        .sender()
        .try_send(tmc4671::TMCCommand::Move(
            STEP_MULT * target_position,
            STEP_MULT as f32 * v0,
            STEP_MULT as f32 * a0,
        ))
        .ok();
    stepper.advance();
}

#[embassy_executor::task]
async fn move_processing(steppers: &'static [ProtectedEmulatedStepper; NUM_STEPPERS]) {
    let mut ticker = tmc4671::TMCTimeIterator::new();
    ticker.set_period(Duration::from_micros(250));
    loop {
        let ticks = ticker.next();
        // Move processing
        for stepper in steppers.iter() {
            stepper.lock(|s| {
                process_moves(s.borrow_mut().deref_mut(), ticks);
            });
        }
        Timer::at(ticks).await
    }
}

#[embassy_executor::task]
async fn usb_comms(
    r: UsbResources,
    steppers: &'static [ProtectedEmulatedStepper; NUM_STEPPERS],
    trsync: &'static [ProtectedTrSync; NUM_TRSYNC],
) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // Enable VBUS detection, OpenFFBoard requires it.
    config.vbus_detection = true;

    Timer::after_millis(100).await;
    info!("Hello USB!");

    let driver: Driver<'_, peripherals::USB_OTG_FS> =
        Driver::new_fs(r.otg, Irqs, r.dplus, r.dminus, &mut ep_out_buffer, config);
    let mut state = usb_anchor::AnchorState::new();
    let in_pipe = usb_anchor::AnchorPipe::new();
    let mut anchor = usb_anchor::UsbAnchor::new();
    let anchor_fut = anchor.run(
        &mut state,
        &in_pipe,
        &USB_OUT_BUFFER,
        driver,
        steppers,
        trsync,
    );
    anchor_fut.await;
    // let anchor_protocol_fut = anchor_protocol(&in_pipe, steppers, trsync);
    // join(anchor_fut, anchor_protocol_fut).await;
}

#[derive(Default)]
pub enum LedState {
    #[default]
    Error,
    Connecting,
    Connected,
    Enabled,
    Waiting,
}

pub static LED_STATE: Signal<CS, LedState> = Signal::new();

#[embassy_executor::task]
async fn tmc_task(r: TmcResources) {
    info!("Hello TMC! {}", DWT::cycle_count());

    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(2_000_000);
    spi_config.gpio_speed = Speed::VeryHigh;

    let spi = spi::Spi::new(r.spi, r.clk, r.mosi, r.miso, r.dma_a, r.dma_b, spi_config);
    let spi_bus = usb_anchor::AnchorMutex::new(spi);
    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let enable = Output::new(r.enable, Level::High, Speed::VeryHigh);
    // let flag = Input::new(r.flag, Pull::Up);
    let brake = Output::new(r.brake, Level::High, Speed::VeryHigh);
    let spi_dev = SpiDevice::new(&spi_bus, cs);
    let mut tmc = tmc4671::TMC4671Async::new_spi(
        spi_dev,
        TMC_CMD.dyn_receiver(),
        // TMC_RESP.dyn_publisher().expect("Initialisation Failure"),
        enable,
        // flag,
        brake,
    );
    LED_STATE.signal(LedState::Error);
    Timer::after_millis(300).await;

    let cfg = TMC4671Config::builder();
    match tmc.init(cfg.build()).await {
        Ok(_) => LED_STATE.signal(LedState::Waiting),
        Err(_) => LED_STATE.signal(LedState::Error),
    }
    tmc.run().await;
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
#[allow(unsafe_op_in_unsafe_fn)]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
#[allow(unsafe_op_in_unsafe_fn)]
unsafe fn UART5() {
    EXECUTOR_MED.on_interrupt()
}

// type OnceLockQei = OnceLock<qei::Qei<'static, TIM3>>;
// static ENCODER: OnceLockQei = OnceLockQei::new();

// #[embassy_executor::task]
// async fn encoder_mon() {
//     TMC_CMD.dyn_sender().send(tmc4671::TMCCommand::Enable).await;
//     let mut old_pos = 0;
//     loop {
// let pos = ENCODER.get().await.count();
// let pos = if pos > 32768 {
//     (pos as i32) - 65536
// } else {
//     pos as i32
// };
//         let pos = pos * 256;
//         let dpos = (pos - old_pos) as f32;
//         old_pos = pos;

//         // info!("Encoder pos {}", pos);
//         TMC_CMD
//             .dyn_sender()
//             .send(tmc4671::TMCCommand::Move(pos, dpos / 0.03, 0.0))
//             .await;
//         Timer::after_millis(10).await;
//     }
// }

#[entry]
fn main() -> ! {
    dfu::maybe_enter_dfu();
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 8mhz / 4 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.mux.clk48sel = mux::Clk48sel::PLL1_Q;
    }
    let p = embassy_stm32::init(config);

    let r = split_resources!(p);

    let stepper_pool = STEPPER_POOL.init([(); NUM_STEPPERS].map(|_| {
        CriticalSectionMutex::new(RefCell::new(stepper::EmulatedStepper::new(
            tmc4671::TMCTimeIterator::new(),
        )))
    }));
    let steppers = STEPPERS.init(stepper_pool);

    let trsync_pool = TRSYNC_POOL
        .init([(); NUM_TRSYNC].map(|_| CriticalSectionMutex::new(RefCell::new(TrSync::new()))));
    let trsync = TRSYNC.init(trsync_pool);

    // Enable the DWT cycle counter.
    {
        let mut peripherals = cortex_m::Peripherals::take().unwrap();
        peripherals.DCB.enable_trace();
        peripherals.DWT.enable_cycle_counter();
    }

    info!("Hello World! {}", DWT::cycle_count());

    // let _ = ENCODER.init(qei::Qei::new(
    //     r.encoder.timer,
    //     qei::QeiPin::new(r.encoder.enc_a),
    //     qei::QeiPin::new(r.encoder.enc_b),
    // ));

    /*
    STM32s don’t have any interrupts exclusively for software use, but they can all be triggered by software as well as
    by the peripheral, so we can just use any free interrupt vectors which aren’t used by the rest of the application.
    In this case we’re using UART4 and UART5, but there’s nothing special about them. Any otherwise unused interrupt
    vector would work exactly the same.
    */

    /*
    High-priority executor: UART4, priority level 6
    TMC control code goes here.
    */
    interrupt::UART4.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::UART4);
    spawner.spawn(tmc_task(r.tmc).expect("Spawn failure"));
    spawner.spawn(move_processing(steppers).expect("Spawn failure"));
    spawner.spawn(trsync_processing(trsync).expect("Spawn failure"));

    // Medium-priority executor: UART5, priority level 7
    interrupt::UART5.set_priority(Priority::P7);
    let spawner = EXECUTOR_MED.start(interrupt::UART5);
    // spawner.spawn(encoder_mon().expect("Spawn failure"));
    spawner.spawn(usb_comms(r.usb, steppers, trsync).expect("Spawn failure"));

    /*
    Low priority executor: runs in thread mode, using WFE/SEV
    */
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(blink(r.led).expect("Spawn failure"));
    });
}
