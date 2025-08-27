#![no_std]
use embedded_interfaces::TransportError;
use embedded_interfaces::registers::RegisterInterfaceAsync;
pub use embedded_interfaces::spi::SpiDeviceAsync;
use registers::*;

use embassy_time::{Duration, Instant};

pub use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{channel, pubsub, watch::Watch};
pub use embedded_hal_async::spi;

pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
pub type TMCCommandChannel = channel::Channel<CS, TMCCommand, 2>;
pub type TMCCommandSender<'a> = channel::DynamicSender<'a, TMCCommand>;
pub type TMCCommandReceiver<'a> = channel::DynamicReceiver<'a, TMCCommand>;
pub type TMCResponseBus = pubsub::PubSubChannel<CS, TMCCommandResponse, 1, 1, 1>;
pub type TMCResponsePublisher<'a> = pubsub::DynPublisher<'a, TMCCommandResponse>;
pub type TMCResponseSubscriber<'a> = pubsub::DynSubscriber<'a, TMCCommandResponse>;

pub mod registers;

static TMC_PERIOD_WATCH: Watch<CS, Duration, 2> = Watch::new_with(Duration::from_hz(125_000));

#[derive(Debug, defmt::Format)]
pub struct TimeIterator {
    next: Instant,
}

impl TimeIterator {
    pub fn new() -> TimeIterator {
        Self {
            next: Instant::now(),
        }
    }

    pub fn reset(&mut self) {
        self.next = Instant::now();
        self.advance();
    }

    pub fn advance(&mut self) -> Instant {
        self.next += TMC_PERIOD_WATCH.try_get().unwrap();
        self.next
    }

    pub fn next(&mut self) -> Instant {
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
pub struct TMC4671<'a, I>
where
    I: embedded_hal_async::spi::SpiDevice,
{
    /// The interface to communicate with the device
    interface: embedded_interfaces::spi::SpiDeviceAsync<I>,
    command_rx: TMCCommandReceiver<'a>,
    response_tx: TMCResponsePublisher<'a>,
}

impl<'a, I> TMC4671<'a, I>
where
    I: embedded_hal_async::spi::SpiDevice,
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
    ) -> Self {
        Self {
            interface: embedded_interfaces::spi::SpiDeviceAsync::new(spi),
            command_rx: command_rx,
            response_tx: response_tx,
        }
    }
}

pub trait TMC4671Register {}

impl<'a, I> TMC4671<'a, I>
where
    I: embedded_hal_async::spi::SpiDevice,
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
        loop {
            match self.command_rx.receive().await {
                TMCCommand::Enable => (),
                TMCCommand::Disable => (),
                TMCCommand::SpiSend(data) => {
                    let _ = self.interface.interface.write(&data).await;
                }
                TMCCommand::SpiTransfer(data) => {
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
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
