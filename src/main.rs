#![no_std]
#![no_main]

use assign_resources::assign_resources;
use cortex_m_rt::entry;
use defmt::*;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_executor::{Executor, InterruptExecutor};
use embassy_futures::{block_on, join::join};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, spi, usb, Config, Peri};
use embassy_sync::{mutex::Mutex, watch::Watch};
use embassy_time::{Instant, Timer};
use static_cell::StaticCell;

use tmc4671;
use {defmt_rtt as _, panic_probe as _};
mod usb_anchor;
use anchor::*;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

assign_resources! {
    led: LedResources {
        led: PD7,
    }
    tmc: TmcResources {
        miso: PA6,
        mosi: PA7,
        clk: PA5,
        cs: PA4,
        spi: SPI1,
        dma_a: DMA2_CH3,
        dma_b: DMA2_CH0,
        errled: PE1,
    }
    usb: UsbResources {
        otg: USB_OTG_FS,
        dplus: PA12,
        dminus: PA11,
    }
}

klipper_enumeration!(
    enum spi_bus {
        spi1,
    }
);

klipper_enumeration!(
    enum pin {
        spi1_cs,
    }
);

#[klipper_constant]
const BUS_PINS_spi1: &str = "spi1_miso,spi1_clk,spi1_mosi";

#[klipper_constant]
const CLOCK_FREQ: u32 = 1_000_000;

#[klipper_command]
pub fn get_uptime(_context: &mut crate::State) {
    let c = Instant::now().as_ticks();
    debug!("uptime");
    klipper_reply!(
        uptime,
        high: u32 = (c >> 32) as u32,
        clock: u32 = (c & 0xFFFF_FFFF) as u32
    );
}

#[klipper_command]
pub fn get_clock() {
    klipper_reply!(clock, clock: u32 = (Instant::now().as_ticks() & 0xFFFF_FFFF) as u32);
    debug!("clock");
}

#[klipper_command]
pub fn emergency_stop() {}

#[klipper_command]
pub fn get_config(context: &State) {
    let crc = context.config_crc;
    debug!("get_config");
    klipper_reply!(
        config,
        is_config: bool = crc.is_some(),
        crc: u32 = crc.unwrap_or(0),
        is_shutdown: bool = false,
        move_count: u16 = 0
    );
}

#[klipper_command]
pub fn config_reset(context: &mut State) {
    debug!("config_reset");
    context.config_crc = None;
}

#[klipper_command]
pub fn finalize_config(context: &mut State, crc: u32) {
    debug!("finalize_config");
    context.config_crc = Some(crc);
}

#[klipper_command]
pub fn allocate_oids(_count: u8) {}

#[klipper_constant]
const MCU: &str = "k4671_openffboard";

#[klipper_constant]
const STATS_SUMSQ_BASE: u32 = 256;

#[klipper_command]
pub fn config_spi_shutdown(_context: &mut State, _oid: u8, _spi_oid: u8, _shutdown_msg: &[u8]) {}

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

#[klipper_command]
pub fn reset() {
    cortex_m::peripheral::SCB::sys_reset();
}

pub struct State {
    config_crc: Option<u32>,
}

impl State {
    pub fn new() -> Self {
        Self { config_crc: None }
    }
}

pub static USB_OUT_BUFFER: usb_anchor::AnchorPipe = usb_anchor::AnchorPipe::new();

pub(crate) struct BufferTransportOutput;

impl TransportOutput for BufferTransportOutput {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        trace!("Output from Anchor!");
        f(&mut scratch);
        let output = scratch.result();
        if let Ok(n) = USB_OUT_BUFFER.try_write(output) {
            if n < output.len() {
                // Retry, possible a ring buffer wrap
                let _ = USB_OUT_BUFFER.try_write(&output[n..]);
            }
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

async fn anchor_protocol(pipe: &usb_anchor::AnchorPipe) {
    let mut state = State::new();
    type RxBuf = FifoBuffer<{ (usb_anchor::MAX_PACKET_SIZE * 2) as usize }>;
    let mut receiver_buf: RxBuf = RxBuf::new();
    let mut rx: [u8; usb_anchor::MAX_PACKET_SIZE as usize] =
        [0; usb_anchor::MAX_PACKET_SIZE as usize];
    loop {
        let n = pipe.read(&mut rx).await;
        receiver_buf.extend(&rx[..n]);
        trace!("Pipe read for Anchor!");
        let recv_data = receiver_buf.data();
        if !recv_data.is_empty() {
            let mut wrap = SliceInputBuffer::new(recv_data);
            trace!("Data for Anchor!");
            KLIPPER_TRANSPORT.receive(&mut wrap, &mut state);
            let consumed = recv_data.len() - wrap.available();
            if consumed > 0 {
                receiver_buf.pop(consumed);
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_comms(r: UsbResources) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // Enable VBUS detection, OpenFFBoard requires it.
    config.vbus_detection = true;

    let driver = Driver::new_fs(r.otg, Irqs, r.dplus, r.dminus, &mut ep_out_buffer, config);
    let mut state = usb_anchor::AnchorState::new();
    let in_pipe = usb_anchor::AnchorPipe::new();

    let mut anchor = usb_anchor::UsbAnchor::new();
    let anchor_fut = anchor.run(&mut state, &in_pipe, &USB_OUT_BUFFER, driver);
    let anchor_protocol_fut = anchor_protocol(&in_pipe);
    join(anchor_fut, anchor_protocol_fut).await;
}

#[embassy_executor::task]
async fn blink(r: LedResources) {
    info!("Hello Blink!");
    let mut led = Output::new(r.led, Level::High, Speed::Low);
    loop {
        led.set_high();
        Timer::after_millis(300).await;

        led.set_low();
        Timer::after_millis(300).await;
    }
}

#[embassy_executor::task]
async fn tmc_task(r: TmcResources) {
    info!("Hello TMC!");

    let mut spi_config = spi::Config::default();
    spi_config.mode = spi::MODE_3;
    spi_config.frequency = Hertz(1_000_000);

    let spi = spi::Spi::new(r.spi, r.clk, r.mosi, r.miso, r.dma_a, r.dma_b, spi_config);
    let spi_bus = usb_anchor::AnchorMutex::new(spi);
    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let spi_dev = SpiDevice::new(&spi_bus, cs);
    let mut tmc = tmc4671::TMC4671::new_spi(
        spi_dev,
        TMC_CMD.dyn_receiver(),
        TMC_RESP.dyn_publisher().expect("Initialisation Failure"),
    );
    let mut errled = Output::new(r.errled, Level::High, Speed::Low);
    errled.set_high();
    Timer::after_millis(300).await;

    match tmc.init().await {
        Ok(_) => errled.set_low(),
        Err(_) => errled.set_high(),
    }
    tmc.run().await;
}

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_MED: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn UART4() {
    EXECUTOR_HIGH.on_interrupt()
}

#[interrupt]
unsafe fn UART5() {
    EXECUTOR_MED.on_interrupt()
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

    // STM32s don’t have any interrupts exclusively for software use, but they can all be triggered by software as well as
    // by the peripheral, so we can just use any free interrupt vectors which aren’t used by the rest of your application.
    // In this case we’re using UART4 and UART5, but there’s nothing special about them. Any otherwise unused interrupt
    // vector would work exactly the same.

    // High-priority executor: UART4, priority level 6
    interrupt::UART4.set_priority(Priority::P6);
    let spawner = EXECUTOR_HIGH.start(interrupt::UART4);
    unwrap!(spawner.spawn(tmc_task(r.tmc)));

    // Medium-priority executor: UART5, priority level 7
    interrupt::UART5.set_priority(Priority::P7);
    let spawner = EXECUTOR_MED.start(interrupt::UART5);
    unwrap!(spawner.spawn(blink(r.led)));

    // Low priority executor: runs in thread mode, using WFE/SEV
    let executor = EXECUTOR_LOW.init(Executor::new());
    executor.run(|spawner| {
        unwrap!(spawner.spawn(usb_comms(r.usb)));
    });
}
