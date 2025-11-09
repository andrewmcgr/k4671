use core::sync::atomic::{AtomicU8, Ordering};

use defmt::*;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Ticker, Timer};

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

#[derive(Default)]
pub enum LedState {
    #[default]
    Error,
    Connecting,
    Connected,
    Enabled,
    Waiting,
    N(u8),
}

pub struct LedSignal {
    state: AtomicU8,
}

pub static LED_STATE: LedSignal = LedSignal {
    state: AtomicU8::new(4),
};

impl LedSignal {
    pub fn signal(&self, state: LedState) {
        let val = match state {
            LedState::Error => 4,
            LedState::Waiting => 0,
            LedState::Connecting => 0,
            LedState::Connected => 0,
            LedState::Enabled => 0,
            LedState::N(n) => n,
        };
        self.state.store(val, Ordering::Relaxed);
    }

    pub fn try_take(&self) -> Option<LedState> {
        let val = self.state.swap(255, Ordering::Relaxed);
        match val {
            0 => Some(LedState::Waiting),
            1 => Some(LedState::Connected),
            2 => Some(LedState::Enabled),
            3 => Some(LedState::Connecting),
            4 => Some(LedState::Error),
            255 => None,
            n => Some(LedState::N(n)),
        }
    }
}


#[embassy_executor::task]
pub async fn blink(r: crate::LedResources) {
    info!("Hello Blink!");
    let mut led = Output::new(r.led, Level::High, Speed::Low);
    let mut focled = Output::new(r.focled, Level::High, Speed::Low);
    let mut errled = Output::new(r.errled, Level::High, Speed::Low);
    let mut current = LedState::Error;
    let mut l: u8 = 0;
    let mut t = Ticker::every(Duration::from_hz(20));
    loop {
        t.next().await;
        if let Some(newstate) = LED_STATE.try_take() {
            current = newstate;
            t.reset_after(Duration::from_millis(200));
        }
        match current {
            LedState::Error => {
                // l = 7;
            }
            LedState::Waiting => {
                // l = 6;
            }
            LedState::Connecting => {
                // l = 3;
            }
            LedState::Connected => {
                // l = 5;
            }
            LedState::Enabled => {
                // l = 7;
            }
            LedState::N(n) => {
                l = n;
            }
        }
        led.set_level((l & 1 != 0).into());
        focled.set_level((l & 2 != 0).into());
        errled.set_level((l & 4 != 0).into());
    }
}

// #[embassy_executor::task]
// pub async fn blink(r: crate::LedResources) {
//     info!("Hello Blink!");
//     let mut led = Output::new(r.led, Level::High, Speed::Low);
//     let mut focled = Output::new(r.focled, Level::High, Speed::Low);
//     let mut errled = Output::new(r.errled, Level::High, Speed::Low);
//     let mut current = LedState::Error;
//     loop {
//         if let Some(newstate) = LED_STATE.try_take() {
//             current = newstate;
//         }
//         match current {
//             LedState::Error => {
//                 led.set_low();
//                 focled.set_low();
//                 errled.set_high();
//                 Timer::after_millis(300).await;
//                 errled.set_low();
//                 Timer::after_millis(300).await;
//             }
//             LedState::Waiting => {
//                 led.set_high();
//                 focled.set_low();
//                 errled.set_low();
//                 Timer::after_millis(300).await;
//                 led.set_low();
//                 Timer::after_millis(300).await;
//             }
//             LedState::Connecting => {
//                 led.set_high();
//                 focled.set_high();
//                 errled.set_low();
//                 Timer::after_millis(300).await;
//                 led.set_low();
//                 focled.set_low();
//                 Timer::after_millis(300).await;
//             }
//             LedState::Connected => {
//                 led.set_low();
//                 focled.set_high();
//                 errled.set_low();
//                 Timer::after_millis(300).await;
//                 focled.set_low();
//                 Timer::after_millis(300).await;
//             }
//             LedState::Enabled => {
//                 led.set_low();
//                 focled.set_high();
//                 errled.set_low();
//                 Timer::after_millis(300).await;
//                 focled.set_low();
//                 led.set_high();
//                 Timer::after_millis(300).await;
//             }
//         }
//     }
// }
