use defmt::*;

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;

use crate::{LED_STATE, LedResources, LedState};

#[embassy_executor::task]
pub async fn blink(r: LedResources) {
    info!("Hello Blink!");
    let mut led = Output::new(r.led, Level::High, Speed::Low);
    let mut focled = Output::new(r.focled, Level::High, Speed::Low);
    let mut errled = Output::new(r.errled, Level::High, Speed::Low);
    let mut current = LedState::Error;
    loop {
        if let Some(newstate) = LED_STATE.try_take() {
            current = newstate;
        }
        match current {
            LedState::Error => {
                led.set_low();
                focled.set_low();
                errled.set_high();
                Timer::after_millis(300).await;
                errled.set_low();
                Timer::after_millis(300).await;
            }
            LedState::Waiting => {
                led.set_high();
                focled.set_low();
                errled.set_low();
                Timer::after_millis(300).await;
                led.set_low();
                Timer::after_millis(300).await;
            }
            LedState::Connecting => {
                led.set_high();
                focled.set_high();
                errled.set_low();
                Timer::after_millis(300).await;
                led.set_low();
                focled.set_low();
                Timer::after_millis(300).await;
            }
            LedState::Connected => {
                led.set_low();
                focled.set_high();
                errled.set_low();
                Timer::after_millis(300).await;
                focled.set_low();
                Timer::after_millis(300).await;
            }
            LedState::Enabled => {
                led.set_low();
                focled.set_high();
                errled.set_low();
                Timer::after_millis(300).await;
                focled.set_low();
                led.set_high();
                Timer::after_millis(300).await;                
            }
        }
    }
}
