#![no_std]
#![no_main]

use assign_resources::assign_resources;
use cortex_m_rt::entry;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_futures::join::join;
pub use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::peripherals::TIM3;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::qei;
use embassy_stm32::usb::Driver;
use embassy_stm32::{Config, Peri, bind_interrupts, peripherals, spi, usb};
use embassy_sync::{once_lock::OnceLock, signal::Signal};
use embassy_time::{Instant, TICK_HZ, Timer};
use static_cell::StaticCell;

use anchor::*;
use tmc4671::{self, CS, TimeIterator, config::TMC4671Config};
use {defmt_rtt as _, panic_probe as _};
mod commands;
mod leds;
// mod spi_passthrough;
mod stepper;
mod stepper_commands;
mod target_queue;
mod usb_anchor;
use crate::leds::blink;

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

pub type EmulatedStepper = stepper::EmulatedStepper<tmc4671::TMCTimeIterator, 256>;

pub struct State {
    config_crc: Option<u32>,
    stepper: EmulatedStepper,
    target_queue: crate::stepper::TargetQueue,
}

impl State {
    pub fn new() -> Self {
        Self {
            config_crc: None,
            stepper: EmulatedStepper::new(tmc4671::TMCTimeIterator::new()),
            target_queue: crate::stepper::TargetQueue::new(),
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

pub(crate) const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

klipper_config_generate!(
  transport = crate::TRANSPORT_OUTPUT: crate::BufferTransportOutput,
  context = &'ctx mut crate::State,
);

pub static TMC_CMD: tmc4671::TMCCommandChannel = tmc4671::TMCCommandChannel::new();
pub static TMC_RESP: tmc4671::TMCResponseBus = tmc4671::TMCResponseBus::new();

fn process_moves(state: &mut State, next_time: Instant) {
    static mut LAST_POS: i32 = 0;
    let crate::target_queue::ControlOutput {
        position: target_position,
        position_1: c1,
        position_2: c2,
    } = state.target_queue.get_for_control(next_time);
    if target_position != unsafe { LAST_POS } {
        debug!("Target pos {} {}", target_position, next_time.as_ticks());
        unsafe { LAST_POS = target_position };
    }

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
    TMC_CMD
        .sender()
        .try_send(tmc4671::TMCCommand::Move(target_position, v0, a0))
        .ok();
    state.stepper.advance(&mut state.target_queue);
}

async fn anchor_protocol(pipe: &usb_anchor::AnchorPipe) {
    let mut state = State::new();
    type RxBuf = FifoBuffer<{ (usb_anchor::MAX_PACKET_SIZE * 2) as usize }>;
    let mut receiver_buf: RxBuf = RxBuf::new();
    let mut rx: [u8; usb_anchor::MAX_PACKET_SIZE as usize] =
        [0; usb_anchor::MAX_PACKET_SIZE as usize];
    let mut ticker = tmc4671::TMCTimeIterator::new();
    info!("Hello Anchor!");

    loop {
        let ticks = ticker.next();
        // Move processing
        process_moves(&mut state, ticks);
        // Now we're done with moves, check for commands from Klipper
        while let Ok(n) = pipe.try_read(&mut rx) {
            receiver_buf.extend(&rx[..n]);
            let recv_data = receiver_buf.data();
            if !recv_data.is_empty() {
                let mut wrap = SliceInputBuffer::new(recv_data);
                KLIPPER_TRANSPORT.receive(&mut wrap, &mut state);
                let consumed = recv_data.len() - wrap.available();
                if consumed > 0 {
                    receiver_buf.pop(consumed);
                }
            }
        }
        Timer::at(ticks).await
    }
}

#[embassy_executor::task]
async fn usb_comms(r: UsbResources) {
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
    let anchor_fut = anchor.run(&mut state, &in_pipe, &USB_OUT_BUFFER, driver);
    let anchor_protocol_fut = anchor_protocol(&in_pipe);
    join(anchor_fut, anchor_protocol_fut).await;
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
    info!("Hello TMC!");

    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(2_000_000);
    spi_config.gpio_speed = Speed::VeryHigh;

    let spi = spi::Spi::new(r.spi, r.clk, r.mosi, r.miso, r.dma_a, r.dma_b, spi_config);
    let spi_bus = usb_anchor::AnchorMutex::new(spi);
    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let enable = Output::new(r.enable, Level::High, Speed::VeryHigh);
    let flag = Input::new(r.flag, Pull::Up);
    let brake = Output::new(r.brake, Level::High, Speed::VeryHigh);
    let spi_dev = SpiDevice::new(&spi_bus, cs);
    let mut tmc = tmc4671::TMC4671Async::new_spi(
        spi_dev,
        TMC_CMD.dyn_receiver(),
        TMC_RESP.dyn_publisher().expect("Initialisation Failure"),
        enable,
        flag,
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

type OnceLockQei = OnceLock<qei::Qei<'static, TIM3>>;
static ENCODER: OnceLockQei = OnceLockQei::new();

#[embassy_executor::task]
async fn encoder_mon() {
    TMC_CMD.dyn_sender().send(tmc4671::TMCCommand::Enable).await;
    let mut old_pos = 0;
    loop {
        let pos = ENCODER.get().await.count();
        let pos = if pos > 32768 {
            (pos as i32) - 65536
        } else {
            pos as i32
        };
        let pos = pos * 256;
        let dpos = (pos - old_pos) as f32;
        old_pos = pos;

        // info!("Encoder pos {}", pos);
        TMC_CMD
            .dyn_sender()
            .send(tmc4671::TMCCommand::Move(pos, dpos / 0.03, 0.0))
            .await;
        Timer::after_millis(10).await;
    }
}

#[entry]
fn main() -> ! {
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

    info!("Hello World!");

    let _ = ENCODER.init(qei::Qei::new(
        r.encoder.timer,
        qei::QeiPin::new(r.encoder.enc_a),
        qei::QeiPin::new(r.encoder.enc_b),
    ));

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

    // Medium-priority executor: UART5, priority level 7
    interrupt::UART5.set_priority(Priority::P7);
    let spawner = EXECUTOR_MED.start(interrupt::UART5);
    // spawner.spawn(encoder_mon().expect("Spawn failure"));
    spawner.spawn(usb_comms(r.usb).expect("Spawn failure"));

    /*
    Low priority executor: runs in thread mode, using WFE/SEV
    */
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(blink(r.led).expect("Spawn failure"));
    });
}
