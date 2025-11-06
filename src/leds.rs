use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use defmt::*;

use crate::{LED_STATE, LedState, commands::maintain_clock};

// pub async fn blink_one_generic<T: Pin>(led: embassy_stm32::Peri<'static, T>) {
//     let mut led = Output::new(led, Level::High, Speed::Low);
//     loop {
//         led.set_low();
//         Timer::after_millis(250).await;
//         led.set_high();
//         Timer::after_millis(250).await;
//     }
// }

// #[macro_export]
// macro_rules! blink_task {
//     ($name:ident, $pin:expr) => {
//         #[embassy_executor::task]
//         pub async fn $name() {
//             $crate::leds::blink_one_generic($pin).await
//         }
//     };
// }

// // Safety: These aren't used anywhere else.
// blink_task!(blink_led, unsafe {
//     Peripherals::steal().PD7
// });
// blink_task!(blink_focled, unsafe {
//     Peripherals::steal().PE0
// });
// blink_task!(blink_errled, unsafe {
//     Peripherals::steal().PE1
// });


#[embassy_executor::task]
pub async fn blink(r: crate::LedResources) {
    info!("Hello Blink!");
    let mut led = Output::new(r.led, Level::High, Speed::Low);
    let mut focled = Output::new(r.focled, Level::High, Speed::Low);
    let mut errled = Output::new(r.errled, Level::High, Speed::Low);
    let mut current = LedState::Error;
    loop {
        maintain_clock();
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
