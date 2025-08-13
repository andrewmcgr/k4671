#![no_std]
use embedded_devices_derive::forward_register_fns;
use embedded_interfaces::registers::{
    ReadableRegister, Register, RegisterInterfaceAsync, WritableRegister,
};
use embedded_interfaces::TransportError;
use registers::*;

pub mod registers;

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
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct TMC4671<
    D: hal::delay::DelayNs,
    I: embedded_interfaces::registers::RegisterInterfaceAsync,
> {
    /// The delay provider
    delay: D,
    /// The interface to communicate with the device
    interface: I,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D, I> TMC4671<D, embedded_interfaces::spi::SpiDevice<I>>
where
    I: hal::spi::r#SpiDevice,
    D: hal::delay::DelayNs,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.
    ///
    /// The device supports SPI mode 3.
    #[inline]
    pub fn new_spi(delay: D, interface: I) -> Self {
        Self {
            delay,
            interface: embedded_interfaces::spi::SpiDevice::new(interface),
        }
    }
}

pub trait TMC4671Register {}

#[forward_register_fns]
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterfaceAsync>
    TMC4671<D, I>
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

}

#[cfg(test)]
mod tests {
    use super::*;
}
