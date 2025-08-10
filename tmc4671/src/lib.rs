#![no_std]

pub mod registers;

/// The TMC 4671 is a hardware Field Oriented Control motor driver.
///
/// For a full description and usage examples, refer to the [module documentation](self).
#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), RegisterInterface),
    sync(feature = "sync"),
    async(feature = "async")
)]
pub struct TMC4671<D: hal::delay::DelayNs, I: embedded_interfaces::registers::RegisterInterface> {
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

#[cfg(test)]
mod tests {
    use super::*;
}
