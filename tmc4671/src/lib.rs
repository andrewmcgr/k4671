#![no_std]
use embedded_devices_derive::forward_register_fns;
use embedded_interfaces::registers::RegisterInterfaceAsync;
use embedded_interfaces::TransportError;
pub use embedded_interfaces::spi::SpiDeviceAsync;
use registers::*;

use embassy_sync::{channel, pubsub};
pub use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embedded_hal_async::spi;

pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
pub type TMCCommandChannel = channel::Channel<CS, TMCCommand, 2>;
pub type TMCResponseBus = pubsub::PubSubChannel<CS, TMCCommandResponse, 1, 1, 1>;

pub mod registers;

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
pub struct TMC4671<
    I: embedded_interfaces::registers::RegisterInterfaceAsync,
> {
    /// The interface to communicate with the device
    interface: I,
    command_channel: TMCCommandChannel,
    response_channel: TMCResponseBus,
}

impl<I> TMC4671<embedded_interfaces::spi::SpiDeviceAsync<I>>
where
    I: spi::SpiDevice,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The device supports SPI mode 3.
    #[inline]
    pub fn new_spi(interface: I) -> Self {
        Self {
            interface: embedded_interfaces::spi::SpiDeviceAsync::new(interface),
            command_channel: TMCCommandChannel::new(),
            response_channel: TMCResponseBus::new(),
        }
    }
}

pub trait TMC4671Register {}

#[forward_register_fns]
impl<I> TMC4671<I>
where
    I: spi::SpiDevice + RegisterInterfaceAsync,
{
    //// Detect a device
    pub async fn init(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        self.write_register(ChipinfoAddr::default().with_value(Chipinfo::ChipinfoSiType))
            .await?;
        let res = self.read_register::<ChipinfoData>().await?;
        if res.read_value() == registers::DEVICE_ID_VALID {
            return Ok(());
        } else {
            return Err(FaultDetectionError::FaultDetected);
        }
    }

    // Run device. Never returns.
    pub async fn run(&mut self) -> ! {
        let command_receiver = self.command_channel.receiver();
        let publisher = self.response_channel.publisher().unwrap();
        loop {
            match command_receiver.receive().await {
                TMCCommand::Enable => (),
                TMCCommand::Disable => (),
                TMCCommand::SpiSend(data) => {
                    let _ = self.interface.write(&data).await;
                }
                TMCCommand::SpiTransfer(data) => {
                    let mut resp: [u8; 5] = [0; 5];
                    if self.interface.transfer(&mut resp, &data).await.is_ok() {
                        publisher.publish_immediate(TMCCommandResponse::SpiResponse(resp));
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
