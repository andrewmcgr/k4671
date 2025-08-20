#![no_std]
#![no_main]

use assign_resources::assign_resources;
use core::cell::RefCell;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, spi, usb, Config, Peri};
use embassy_sync::mutex;
use embassy_sync::pipe::{Pipe, Reader, Writer};
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;

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

pub struct State {
    config_crc: Option<u32>,
}

impl State {
    pub fn new() -> Self {
        Self {
            config_crc: None,
        }
    }
}

const ANCHOR_PIPE_SIZE: usize = 1024;
type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
type Mutex<T> = embassy_sync::blocking_mutex::Mutex<CS, T>;

pub static USB_TX_BUFFER: Mutex<RefCell<FifoBuffer<{ ANCHOR_PIPE_SIZE }>>> =
    Mutex::new(RefCell::new(FifoBuffer::new()));

pub(crate) struct BufferTransportOutput;

impl TransportOutput for BufferTransportOutput {
    type Output = ScratchOutput;
    fn output(&self, f: impl FnOnce(&mut Self::Output)) {
        let mut scratch = ScratchOutput::new();
        f(&mut scratch);
        let output = scratch.result();
        critical_section::with(|cs| USB_TX_BUFFER.borrow(cs).borrow_mut().extend(output));
    }
}

pub(crate) const TRANSPORT_OUTPUT: BufferTransportOutput = BufferTransportOutput;

klipper_config_generate!(
  transport = crate::TRANSPORT_OUTPUT: crate::BufferTransportOutput,
  context = &'ctx mut crate::State,
);

type RxBuf = FifoBuffer<{ (usb_anchor::MAX_PACKET_SIZE * 2) as usize }>;
#[embassy_executor::task]
async fn anchor_protocol(pipe: Pipe<CS, { ANCHOR_PIPE_SIZE }>) {
    let mut state = State::new();
    let mut receiver_buf: RxBuf = RxBuf::new();
    loop {
        let _ = pipe.read(receiver_buf.receive_buffer()).await;
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

    let mut anchor: usb_anchor::UsbAnchor = usb_anchor::UsbAnchor::new();
    anchor.run(&mut state, driver).await;
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
    let cs = Output::new(r.cs, Level::High, Speed::VeryHigh);
    let spi = unwrap!(ExclusiveDevice::new(spi, cs, Delay));
    let mut tmc = tmc4671::TMC4671Async::new_spi(Delay, spi);
    let mut errled = Output::new(r.errled, Level::High, Speed::Low);
    errled.set_high();
    Timer::after_millis(300).await;

    match tmc.init().await {
        Ok(_) => errled.set_low(),
        Err(_) => errled.set_high(),
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
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

    spawner.must_spawn(tmc_task(r.tmc));
    spawner.must_spawn(blink(r.led));
    spawner.must_spawn(usb_comms(r.usb));
}
