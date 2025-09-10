#![no_std]
use defmt::*;
use embedded_devices_derive::forward_register_fns;
use embedded_interfaces::TransportError;
use embedded_interfaces::registers::RegisterInterfaceAsync;
pub use embedded_interfaces::spi::SpiDeviceAsync;
use registers::*;

use embassy_time::{Duration, Instant, Timer};

pub use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{channel, pubsub};
pub use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal_async::spi;

use const_builder::ConstBuilder;

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
            advance: Duration::from_hz(15000),
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
    // SpiSend([u8; 5]),
    // SpiTransfer([u8; 5]),
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

#[derive(ConstBuilder)]
#[builder(default)]
pub struct TMC4671Config {
    #[builder(default = 43.64)]
    voltage_scale: f32,
    #[builder(default = 15e3)]
    pwm_freq_target: f32,
    #[builder(default = 2)]
    phases: u8,
    #[builder(default = 50)]
    n_pole_pairs: u8,
    #[builder(default = 10)]
    pwm_bbm_l: u32,
    #[builder(default = 10)]
    pwm_bbm_h: u32,
    #[builder(default = 7)]
    pwm_chop: u32,
    #[builder(default = 1)]
    pwm_sv: u32,
    #[builder(default = 3)]
    motor_type: u32,
    #[builder(default = 0)]
    adc_i_ux_select: u32,
    #[builder(default = 2)]
    adc_i_v_select: u32,
    #[builder(default = 1)]
    adc_i_wy_select: u32,
    #[builder(default = 0)]
    adc_i0_select: u32,
    #[builder(default = 1)]
    adc_i1_select: u32,
    #[builder(default = 1)] // 120 degree analog hall
    aenc_deg: u32,
    #[builder(default = 1)] // 120 degree analog hall
    aenc_ppr: u32,
    #[builder(default = 0)]
    abn_apol: u32,
    #[builder(default = 0)]
    abn_bpol: u32,
    #[builder(default = 0)]
    abn_npol: u32,
    #[builder(default = 0)]
    abn_use_abn_as_n: u32,
    #[builder(default = 0)]
    abn_cln: u32,
    #[builder(default = 0)]
    abn_direction: u32,
    #[builder(default = 4000)]
    abn_decoder_ppr: u32,
    #[builder(default = 0)]
    hall_interp: u32,
    #[builder(default = 1)]
    hall_sync: u32,
    #[builder(default = 0)]
    hall_polarity: u32,
    #[builder(default = 0)]
    hall_dir: u32,
    #[builder(default = 0xAAAA)]
    hall_dphi_max: u32,
    #[builder(default = 0)]
    hall_phi_e_offset: u32,
    #[builder(default = 2)]
    hall_blank: u32,
    #[builder(default = 3)]
    phi_e_selection: u32,
    #[builder(default = 9)]
    position_selection: u32,
    #[builder(default = 3)]
    velocity_selection: u32,
    #[builder(default = 1)] // PWM frequency velocity meter
    velocity_meter_selection: u32,
    #[builder(default = 0)] // Advanced PID samples position at fPWM
    mode_pid_smpl: u32,
    #[builder(default = 1)] // Advanced PID mode
    mode_pid_type: u32,
    #[builder(default = 31500)] // Voltage limit, 32768 = Vm
    pidout_uq_ud_limits: u32,
    #[builder(default=-0x10000000)]
    pid_position_limit_low: i32,
    #[builder(default = 0x10000000)]
    pid_position_limit_high: i32,
    #[builder(default = 0x10000000)]
    pid_velocity_limit: u32,
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
pub struct TMC4671<'a, I, O, P>
where
    I: embedded_interfaces::registers::RegisterInterface,
    O: OutputPin,
    P: InputPin,
{
    enabled: bool,
    /// The interface to communicate with the device
    interface: I,
    command_rx: TMCCommandReceiver<'a>,
    response_tx: TMCResponsePublisher<'a>,
    enable_pin: O,
    flag_pin: P,
    brake_pin: O,
}

#[maybe_async_cfg::maybe(
    idents(hal(sync = "embedded_hal", async = "embedded_hal_async"), SpiDevice),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<'a, I, O, P> TMC4671<'a, embedded_interfaces::spi::SpiDevice<I>, O, P>
where
    I: hal::spi::r#SpiDevice,
    O: OutputPin,
    P: InputPin,
{
    /// Initializes a new device from the specified SPI device.
    /// This consumes the SPI device `I`.``
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
            enabled: false,
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

macro_rules! reg {
    ($($self:ident. $reg:ident[$addr_reg:ident->$addr:expr] = $e:expr);+) => {
        $(
        let _ = $self
            .write_register($addr_reg::default().with_value($addr)).await;
        let _ = $self
            .write_register($reg::default().with_value($e)).await;
        )+
    };
    ($($self:ident. $reg:ident = $e:expr);+) => {
        $(
        let _ = $self
            .write_register($reg::default().with_value($e)).await;
        )+
    };
    ($self:ident. $reg:ident[$addr_reg:ident->$addr:expr]) => {
        {
            let _ = $self
                .write_register($addr_reg::default().with_value($addr)).await;
            $self.read_register::<$reg>()
        }
    };
    ($self:ident. $reg:ident) => {
        $self.read_register::<$reg>()
    };
}

#[forward_register_fns]
#[maybe_async_cfg::maybe(
    idents(
        hal(sync = "embedded_hal", async = "embedded_hal_async"),
        RegisterInterface
    ),
    sync(feature = "sync"),
    async(feature = "async")
)]
impl<'a, I, O, P> TMC4671<'a, I, O, P>
where
    I: embedded_interfaces::registers::RegisterInterface,
    O: OutputPin,
    P: InputPin,
{
    //// Detect a device
    pub async fn ident(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        let _ = self
            .write_register(ChipinfoAddr::default().with_chipinfo_addr(Chipinfo::ChipinfoSiType))
            .await;
        let typ = self
            .read_register::<ChipinfoSiType>()
            .await?
            .read_chipinfo_si_type();
        if typ != DEVICE_ID_VALID {
            return Err(FaultDetectionError::FaultDetected);
        }
        let _ = self
            .write_register(ChipinfoAddr::default().with_chipinfo_addr(Chipinfo::ChipinfoSiVersion))
            .await;
        let ver = self.read_register::<ChipinfoSiVersion>().await?;
        let _ = self
            .write_register(ChipinfoAddr::default().with_chipinfo_addr(Chipinfo::ChipinfoSiDate))
            .await;
        let date = self.read_register::<ChipinfoSiDate>().await?;

        info!(
            "TMC Detected {:x}, v{}.{}, {:x}",
            typ,
            ver.read_chipinfo_si_version_hi(),
            ver.read_chipinfo_si_version_lo(),
            date.read_chipinfo_si_date()
        );
        Ok(())
    }

    pub async fn init(
        &mut self,
        cfg: TMC4671Config,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        let _ = self.ident().await?;
        Ok(())
        // Configure the device according to cfg
    }

    // Run device. Never returns.
    pub async fn run(&mut self) -> ! {
        let mut ticker = TMCTimeIterator::new();
        loop {
            while let Some(cmd) = self.command_rx.try_receive().ok() {
                match cmd {
                    TMCCommand::Enable => {
                        info!("TMC Command {}", cmd);
                        self.enabled = true;
                        let _ = self.enable_pin.set_high();
                    }
                    TMCCommand::Disable => {
                        info!("TMC Command {}", cmd);
                        self.enabled = false;
                        let _ = self.enable_pin.set_low();
                    }
                    // TMCCommand::SpiSend(data) => {
                    //     info!("TMC Command {:x}", cmd);
                    //     let _ = self.interface.write(&data).await;
                    // }
                    // TMCCommand::SpiTransfer(data) => {
                    //     info!("TMC Command {:x}", cmd);
                    //     let mut resp: [u8; 5] = [0; 5];
                    //     if self
                    //         .interface
                    //         .interface
                    //         .transfer(&mut resp, &data)
                    //         .await
                    //         .is_ok()
                    //     {
                    //         let r = TMCCommandResponse::SpiResponse(resp);
                    //         info!("TMC Responds {:x}", r);
                    //         self.response_tx.publish_immediate(r);
                    //     }
                    // }
                    TMCCommand::Move(pos, _vel, _accel) => {
                        if self.enabled {
                            self.write_register(
                                PidPositionTarget::default().with_pid_position_target(pos),
                            )
                            .await
                            .ok();
                        }
                    }
                }
            }
            let ticks = ticker.next();
            Timer::at(ticks).await
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
}
