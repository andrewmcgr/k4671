#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{Config, Spi, MODE_3};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use tmc4671;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut spi_config = Config::default();
    spi_config.mode = MODE_3;
    spi_config.frequency = Hertz(1_000_000);

    let mut spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, spi_config);

    let mut cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);

    let mut errled = Output::new(p.PE1, Level::High, Speed::Low);
    let mut led = Output::new(p.PD7, Level::High, Speed::Low);

    errled.set_low();
    let mut buf = [0x81u8, 0x0, 0x0, 0x0, 0x0];
    cs.set_low();
    unwrap!(spi.blocking_transfer_in_place(&mut buf));
    cs.set_high();
    info!("xfer {=[u8]:x}", buf);
    buf = [0x0; 5];
    cs.set_low();
    unwrap!(spi.blocking_transfer_in_place(&mut buf));
    cs.set_high();
    info!("xfer {=[u8]:x}", buf);
    if buf != [0x0, 0x34, 0x36, 0x37, 0x31] {
        errled.set_high();
        info!("Chip ID failed!");
    }

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(300).await;

        info!("low");
        led.set_low();
        Timer::after_millis(300).await;
    }
}
