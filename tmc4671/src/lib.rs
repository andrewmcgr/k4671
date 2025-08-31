#![no_std]
use defmt::*;
use embedded_interfaces::TransportError;
use embedded_interfaces::registers::RegisterInterfaceAsync;
pub use embedded_interfaces::spi::SpiDeviceAsync;
use registers::*;

use embassy_time::{Duration, Instant, Timer};

pub use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{channel, pubsub};
pub use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal_async::spi;

pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type TMCCommandChannel = channel::Channel<CS, TMCCommand, 2>;
pub type TMCCommandSender<'a> = channel::DynamicSender<'a, TMCCommand>;
pub type TMCCommandReceiver<'a> = channel::DynamicReceiver<'a, TMCCommand>;
pub type TMCResponseBus = pubsub::PubSubChannel<CS, TMCCommandResponse, 1, 1, 1>;
pub type TMCResponsePublisher<'a> = pubsub::DynPublisher<'a, TMCCommandResponse>;
pub type TMCResponseSubscriber<'a> = pubsub::DynSubscriber<'a, TMCCommandResponse>;

pub mod registers;

pub trait TimeIterator {
    fn next(&mut self) -> Instant;
    fn advance(&mut self) -> Instant;
}

#[derive(Debug, defmt::Format)]
pub struct TMCTimeIterator {
    next: Instant,
    advance: Duration,
}

impl TMCTimeIterator {
    pub fn new() -> TMCTimeIterator {
        Self {
            next: Instant::now(),
            advance: Duration::from_hz(25000),
        }
    }

    pub fn reset(&mut self) {
        self.next = Instant::now();
        self.advance();
    }

    pub fn set_period(&mut self, t: Duration) {
        self.advance = t;
    }
}

impl TimeIterator for TMCTimeIterator {
    fn advance(&mut self) -> Instant {
        self.next += self.advance;
        self.next
    }

    fn next(&mut self) -> Instant {
        loop {
            if self.next >= Instant::now() {
                return self.next;
            } else {
                self.advance();
            }
        }
    }
}

#[derive(Debug, defmt::Format, Copy, Clone)]
pub enum TMCCommand {
    Enable,
    Disable,
    SpiSend([u8; 5]),
    SpiTransfer([u8; 5]),
    Move(i32, f32, f32),
}

#[derive(Debug, defmt::Format, Copy, Clone)]
pub enum TMCCommandResponse {
    SpiResponse([u8; 5]),
}

#[derive(Debug, defmt::Format, thiserror::Error)]
pub enum FaultDetectionError<BusError> {
    /// Transport error
    #[error("transport error")]
    Transport(#[from] TransportError<(), BusError>),
    /// Timeout (the detection never finished in the allocated time frame)
    #[error("fault detection timeout")]
    Timeout,
    /// A fault was detected. Read the FaultStatus register for details.
    #[error("fault detected")]
    FaultDetected,
}

/// The TMC 4671 is a hardware Field Oriented Control motor driver.
///
/// For a full description and usage examples, refer to the [module documentation](self).
pub struct TMC4671<'a, I, O, P>
where
    I: embedded_hal_async::spi::SpiDevice,
    O: OutputPin,
    P: InputPin,
{
    /// The interface to communicate with the device
    interface: embedded_interfaces::spi::SpiDeviceAsync<I>,
    command_rx: TMCCommandReceiver<'a>,
    response_tx: TMCResponsePublisher<'a>,
    enable_pin: O,
    flag_pin: P,
    brake_pin: O,
}

impl<'a, I, O, P> TMC4671<'a, I, O, P>
where
    I: embedded_hal_async::spi::SpiDevice,
    O: OutputPin,
    P: InputPin,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The device supports SPI mode 3.
    #[inline]
    pub fn new_spi(
        spi: I,
        command_rx: TMCCommandReceiver<'a>,
        response_tx: TMCResponsePublisher<'a>,
        enable_pin: O,
        flag_pin: P,
        brake_pin: O,
    ) -> Self {
        Self {
            interface: embedded_interfaces::spi::SpiDeviceAsync::new(spi),
            command_rx: command_rx,
            response_tx: response_tx,
            enable_pin: enable_pin,
            flag_pin: flag_pin,
            brake_pin: brake_pin,
        }
    }
}

pub trait TMC4671Register {}

impl<'a, I, O, P> TMC4671<'a, I, O, P>
where
    I: embedded_hal_async::spi::SpiDevice,
    O: OutputPin,
    P: InputPin,
{
    //// Detect a device
    pub async fn init(
        &mut self,
    ) -> Result<
        (),
        FaultDetectionError<
            <embedded_interfaces::spi::SpiDeviceAsync<I> as RegisterInterfaceAsync>::BusError,
        >,
    > {
        self.interface
            .write_register(ChipinfoAddr::default().with_value(Chipinfo::ChipinfoSiType))
            .await?;
        let res = self.interface.read_register::<ChipinfoData>().await?;
        if res.read_value() == registers::DEVICE_ID_VALID {
            return Ok(());
        } else {
            return Err(FaultDetectionError::FaultDetected);
        }
    }

    // Run device. Never returns.
    pub async fn run(&mut self) -> ! {
        let mut ticker = TMCTimeIterator::new();
        loop {
            let ticks = ticker.next();
            match self.command_rx.try_receive().ok() {
                Some(TMCCommand::Enable) => {
                    info!("TMC Enable!");
                    let _ = self.enable_pin.set_high();
                }
                Some(TMCCommand::Disable) => {
                    info!("TMC Disable!");
                    let _ = self.enable_pin.set_low();
                }
                Some(TMCCommand::SpiSend(data)) => {
                    info!("TMC SpiSend!");
                    let _ = self.interface.interface.write(&data).await;
                }
                Some(TMCCommand::SpiTransfer(data)) => {
                    info!("TMC SpiTransfer!");
                    let mut resp: [u8; 5] = [0; 5];
                    if self
                        .interface
                        .interface
                        .transfer(&mut resp, &data)
                        .await
                        .is_ok()
                    {
                        self.response_tx
                            .publish_immediate(TMCCommandResponse::SpiResponse(resp));
                    }
                }
                Some(TMCCommand::Move(pos, vel, accel)) => {
                    let _ = self
                        .interface
                        .write_register(PidPositionTarget::default().with_value(pos))
                        .await;
                }
                None => (),
            }
            Timer::at(ticks).await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
