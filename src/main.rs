#![no_std]
#![no_main]

use defmt::*;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals, usb, Config, spi, Peri, uid};
use embassy_time::{Delay, Timer};
use embassy_executor::Spawner;
use embedded_hal_bus::spi::ExclusiveDevice;
use embassy_stm32::usb::Driver;
use tmc4671;
use {defmt_rtt as _, panic_probe as _};
use assign_resources::assign_resources;
mod usb_anchor;

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

#[embassy_executor::task]
async fn usb_comms(r: UsbResources) {
    // Create the driver, from the HAL.
    let mut ep_out_buffer = [0u8; 256];
    let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = true;

    let driver = Driver::new_fs(r.otg, Irqs, r.dplus, r.dminus, &mut ep_out_buffer, config);
    let mut state = usb_anchor::AnchorState::new();

    let anchor: usb_anchor::UsbAnchor<1024, usb_anchor::DummyHandler> = usb_anchor::UsbAnchor::new();
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
