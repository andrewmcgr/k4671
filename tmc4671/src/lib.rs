#![no_std]
#![feature(core_float_math)]

use bytemuck::cast;
use defmt::*;
use embedded_devices_derive::forward_register_fns;
use embedded_interfaces::TransportError;
pub use embedded_interfaces::spi::SpiDeviceAsync;
use registers::*;

use embassy_time::{Duration, Instant, Timer};

pub use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::{channel, pubsub};
pub use embedded_hal::digital::{InputPin, OutputPin};
pub use embedded_hal_async::spi;

use core::f32::math::round;
use fixed::types::{I4F12, I8F8};
use paste::paste;

pub type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub type TMCCommandChannel = channel::Channel<CS, TMCCommand, 2>;
pub type TMCCommandSender<'a> = channel::DynamicSender<'a, TMCCommand>;
pub type TMCCommandReceiver<'a> = channel::DynamicReceiver<'a, TMCCommand>;
pub type TMCResponseBus = pubsub::PubSubChannel<CS, TMCCommandResponse, 1, 1, 1>;
pub type TMCResponsePublisher<'a> = pubsub::DynPublisher<'a, TMCCommandResponse>;
pub type TMCResponseSubscriber<'a> = pubsub::DynSubscriber<'a, TMCCommandResponse>;

pub mod biquad;
pub mod config;
pub mod registers;

use embedded_interfaces::packable::UnsignedPackable;

// The 4671 has a 25 MHz external clock
const TMC_FREQUENCY: f32 = 25000000.0;

const VM_RANGE: u16 = round(32767.0 / 1.25) as u16;

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
    current_scale_ma_lsb: f32,
    current_limit: u16,
    pwmfreq: f32,
    run_current: f32,
    flux_current: f32,
    voltage_scale: f32,
    voltage_scale_i: u16,
    last_pos: i32,
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
    /// The device supports only SPI mode 3.
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
            current_scale_ma_lsb: 1.272,
            current_limit: 0,
            pwmfreq: 50e3,
            run_current: 0.0,
            flux_current: 0.0,
            voltage_scale: 43.64,
            voltage_scale_i: 44,
            last_pos: 0,
        }
    }
}

pub trait TMC4671Register {}

macro_rules! pid_impl {
    ($nom:ident) => {
        paste! {
            pub async fn [<set_ $nom:lower _pid>](&mut self, p: f32, i: f32)
            {
                let val = [<Pid $nom:camel P $nom:camel I> ]::default()
                            .[<with_pid_ $nom:lower _p>](I8F8::from_num(p).to_bits())
                            .[<with_pid_ $nom:lower _i>](I4F12::from_num(i).to_bits());
                info!("TMC Set {} PID: P {}, I {}, Reg {:x}", stringify!($nom), p, i, val.0);
                let _ = self
                    .write_register(
                        val
                    )
                    .await;
            }
        }
    };
}

macro_rules! regdump {
    (($nom:ident),+) => {
        {
            info!("TMC Register Dump:");
            $(
                let reg = self.read_register::<$nom>().await?;
                info!("  {}: {:x}", stringify!($nom), reg.0);
            )+
            info!("End of Register Dump");
        }
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

    pub async fn set_pwm_freq(
        &mut self,
        freq: f32,
    ) -> Result<f32, FaultDetectionError<I::BusError>> {
        let maxcnt = (4.0 * TMC_FREQUENCY / freq) as u32 - 1;
        let pwmfreq = 4.0 * TMC_FREQUENCY / (maxcnt as f32 + 1.0);
        self.pwmfreq = pwmfreq;
        let pwmt_ns = maxcnt + 1;
        let mcnt: u32 = 0x1000_0000; // 12.5 MHz
        let mdec = ((pwmt_ns / (3 * (0x8000_0000 / mcnt))) - 2) as u16;
        info!("TMC PWM Frequency: {} Hz", pwmfreq);
        info!("  maxcnt: {}, mdec: {}, mcnt: {:x}", maxcnt, mdec, mcnt);
        let _ = self
            .write_register(PwmMaxcnt::default().with_pwm_maxcnt(maxcnt))
            .await;
        // Set ADC clock
        let _ = self
            .write_register(DsadcMclkA::default().with_dsadc_mclk_a(mcnt))
            .await;
        let _ = self
            .write_register(DsadcMclkB::default().with_dsadc_mclk_b(mcnt))
            .await;
        let _ = self
            .write_register(
                DsadcMdecBMdecA::default()
                    .with_dsadc_mdec_a(mdec)
                    .with_dsadc_mdec_b(mdec),
            )
            .await;
        Ok(pwmfreq)
    }

    // Set a biquad filter.
    // Argument CF is the address of the enable register
    // fc is the center frequency in Hz
    // fs is the sampling frequency in Hz
    pub async fn set_filter(
        &mut self,
        cf: Config,
        fc: f32,
        fs: f32,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        let bq = biquad::biquad_lpf(fc, fs);
        let bqt = bq.to_tmc();
        info!("TMC Biquad LPF: fc {} Hz, fs {} Hz", fc, fs);
        info!(
            "  b0: {}, b1: {}, b2: {}, a1: {}, a2: {}",
            bq.b0, bq.b1, bq.b2, bq.a1, bq.a2
        );
        info!(
            "  TMC b0: {:x}, b1: {:x}, b2: {:x}, a1: {:x}, a2: {:x}",
            bqt.b0 as u32, bqt.b1 as u32, bqt.b2 as u32, bqt.a1 as u32, bqt.a2 as u32
        );
        let cfi = cf.to_unsigned();
        let l = [bqt.a1, bqt.a2, bqt.b0, bqt.b1, bqt.b2];
        for a in 0..5 {
            self.write_register(
                ConfigAddr::default().with_config_addr(Config::from_unsigned(cfi - 6 + a)),
            )
            .await?;
            self.write_register(ConfigData::default().with_config_data(l[a as usize] as u32))
                .await?;
        }
        self.write_register(ConfigAddr::default().with_config_addr(cf))
            .await?;
        self.write_register(
            ConfigData::default().with_config_data(0xffff_ffff), // not sure which of these is correct
        )
        .await?;
        Ok(())
    }

    pub async fn disable_motor(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        // Turn off the power stage
        let _ = self.enable_pin.set_low();
        self.enabled = false;
        // Set motion mode to stopped
        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::StoppedMode))
            .await;
        // Disable PWM outputs
        let _ = self
            .write_register(PwmSvChop::default().with_pwm_sv(false).with_pwm_chop(0))
            .await;
        Ok(())
    }

    pub async fn enable_motor(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        if self.enabled {
            return Ok(());
        }
        // Set motion mode to stopped
        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::StoppedMode))
            .await;
        // Enable PWM outputs, don't change pwm_sv
        let pwm_sv_chop = self.read_register::<PwmSvChop>().await?;
        let _ = self.write_register(pwm_sv_chop.with_pwm_chop(7)).await;
        // Turn on the power stage
        let _ = self.enable_pin.set_high();
        self.enabled = true;
        Ok(())
    }

    pub async fn sample_adc(&mut self) -> Result<(u16, u16), FaultDetectionError<I::BusError>> {
        let _ = self
            .write_register(AdcRawAddr::default().with_adc_raw_addr(AdcRaw::AdcI1RawAdcI0Raw))
            .await;
        let mut i0: u32 = 0;
        let mut i1: u32 = 0;
        let n: u32 = 127;
        for _ in 0..n {
            let regs = self.read_register::<AdcI1RawAdcI0Raw>().await?;
            i0 += regs.read_adc_i0_raw() as u32;
            i1 += regs.read_adc_i1_raw() as u32;
            Timer::after(Duration::from_micros(1000)).await;
        }
        Ok(((i0 / (n)) as u16, (i1 / (n)) as u16))
    }

    pub async fn sample_vm(&mut self) -> Result<(u16, u16), FaultDetectionError<I::BusError>> {
        let _ = self
            .write_register(AdcRawAddr::default().with_adc_raw_addr(AdcRaw::AdcAgpiARawAdcVmRaw))
            .await;
        let mut vml = u16::MAX;
        let mut vmh = 0u16;
        let n: u32 = 100;
        for _ in 0..n {
            let regs = self.read_register::<AdcAgpiARawAdcVmRaw>().await?;
            let vm = regs.read_adc_vm_raw();
            if vm < vml {
                vml = vm;
            }
            if vm > vmh {
                vmh = vm;
            }
            Timer::after(Duration::from_micros(1000)).await;
        }
        Ok((vmh, vml))
    }

    pub async fn calibrate_adc(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        // Calibrate current ADCs
        let cfg_adc = self.read_register::<DsAnalogInputStageCfg>().await?;
        let _ = self
            .write_register(cfg_adc.with_cfg_adc_i0(0).with_cfg_adc_i1(0))
            .await;
        let reg = self.read_register::<DsAnalogInputStageCfg>().await?;
        info!(
            "TMC ADC Config I0 {}, I1 {}",
            reg.read_cfg_adc_i0(),
            reg.read_cfg_adc_i1()
        );

        let (i0_offset, i1_offset) = self.sample_adc().await?;
        info!("TMC ADC Offsets: {}, {}", i0_offset, i1_offset);
        let _ = self
            .write_register(
                AdcI0ScaleOffset::default()
                    .with_adc_i0_offset(i0_offset)
                    .with_adc_i0_scale(1 << 8),
            )
            .await;
        let _ = self
            .write_register(
                AdcI1ScaleOffset::default()
                    .with_adc_i1_offset(i1_offset)
                    .with_adc_i1_scale(1 << 8),
            )
            .await;
        // Calibrate voltage ADC for brake settings
        let (vmh, vml) = self.sample_vm().await?;
        // info!("TMC VM Range: {}, {}", vmh, vml);
        let vmr = vmh - vml;
        let high: u32 = vmr as u32 / 2 + vmh as u32 + (0.05 * self.voltage_scale + 0.5) as u32;
        // info!("TMC VM Brake Range: {}, {}", high, vmr / 2 + vmh);
        if high < u16::MAX as u32 {
            let _ = self
                .write_register(
                    AdcVmLimits::default()
                        .with_adc_vm_limit_high(high as u16)
                        .with_adc_vm_limit_low(vmr / 2 + vmh),
                )
                .await;
        } else {
            // What else can we do but disable the brake?
            let _ = self
                .write_register(
                    AdcVmLimits::default()
                        .with_adc_vm_limit_high(0)
                        .with_adc_vm_limit_low(0),
                )
                .await;
        }
        let reg = self.read_register::<AdcI0ScaleOffset>().await?;
        info!(
            "TMC ADC I0 Offset {}, Scale {}",
            reg.read_adc_i0_offset(),
            reg.read_adc_i0_scale()
        );
        let reg = self.read_register::<AdcI1ScaleOffset>().await?;
        info!(
            "TMC ADC I1 Offset {}, Scale {}",
            reg.read_adc_i1_offset(),
            reg.read_adc_i1_scale()
        );
        let reg = self.read_register::<AdcVmLimits>().await?;
        info!(
            "TMC ADC VM High {}, Low {}",
            reg.read_adc_vm_limit_high(),
            reg.read_adc_vm_limit_low()
        );
        Ok(())
    }

    fn calculate_flux_limit(&self, c: f32) -> u16 {
        let flux = ((c * 1000.0) / self.current_scale_ma_lsb) as u16;
        if flux > 4095 { 4095 } else { flux }
    }

    pub async fn set_current(&mut self, c: f32) -> Result<(), FaultDetectionError<I::BusError>> {
        let flux = self.calculate_flux_limit(c);
        self.current_limit = flux;
        info!("TMC Flux Current Limit: {} A -> {}", c, flux);
        let _ = self
            .write_register(PidTorqueFluxLimits::default().with_pid_torque_flux_limits(flux))
            .await;
        Ok(())
    }

    pub async fn set_run_current(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        self.set_current(self.run_current).await
    }

    pub async fn set_motion_mode(
        &mut self,
        mode: MotionMode,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        let modereg = self.read_register::<ModeRampModeMotion>().await?;
        let _ = self.write_register(modereg.with_mode_motion(mode)).await;
        Ok(())
    }

    pid_impl!(flux);
    pid_impl!(torque);
    pid_impl!(velocity);
    pid_impl!(position);

    pub async fn get_phi_e(&mut self) -> Result<i16, FaultDetectionError<I::BusError>> {
        let phi_e = self.read_register::<PhiE>().await?;
        Ok(phi_e.read_phi_e())
    }

    pub async fn get_pid_position_actual(
        &mut self,
    ) -> Result<i32, FaultDetectionError<I::BusError>> {
        let pos = self.read_register::<PidPositionActual>().await?;
        Ok(pos.read_pid_position_actual())
    }

    pub async fn get_adc_currents(
        &mut self,
    ) -> Result<(i16, i16, i16), FaultDetectionError<I::BusError>> {
        let i_u = self.read_register::<AdcIwyIux>().await?;
        let i_v = self.read_register::<AdcIv>().await?;
        // info!("TMC ADC Currents Raw: {:x}, {:x}", i_u.0, i_v.0);
        Ok((i_u.read_adc_iux(), i_u.read_adc_iwy(), i_v.read_adc_iv()))
    }

    pub async fn get_raw_adc_currents(
        &mut self,
    ) -> Result<(u16, u16), FaultDetectionError<I::BusError>> {
        let _ = self
            .write_register(AdcRawAddr::default().with_adc_raw_addr(AdcRaw::AdcI1RawAdcI0Raw))
            .await;
        let regs = self.read_register::<AdcI1RawAdcI0Raw>().await?;
        Ok((regs.read_adc_i0_raw(), regs.read_adc_i1_raw()))
    }

    pub async fn init(
        &mut self,
        cfg: config::TMC4671Config,
    ) -> Result<(), FaultDetectionError<I::BusError>> {
        // Check that we have a device
        let _ = self.ident().await?;

        // Disable the motor
        self.disable_motor().await?;
        self.brake_pin.set_low().ok();

        // Save configuration parameters
        self.current_scale_ma_lsb = cfg.current_scale_ma_lsb;
        self.run_current = cfg.run_current;
        self.flux_current = cfg.flux_current;
        self.voltage_scale = cfg.voltage_scale;
        self.voltage_scale_i = round(self.voltage_scale) as u16;

        // Static configuration
        let _ = self
            .write_register(
                ConfigAdvancedPiRepresent::default()
                    .with_config_current_i(true)
                    .with_config_current_p(false)
                    .with_config_velocity_i(true)
                    .with_config_velocity_p(false)
                    .with_config_position_i(true)
                    .with_config_position_p(false),
            )
            .await;

        // Configure the device according to cfg

        // Set the PID parameters
        let _ = self
            .set_flux_pid(cfg.pid_flux_p_i.0, cfg.pid_flux_p_i.1)
            .await;
        let _ = self
            .set_torque_pid(cfg.pid_torque_p_i.0, cfg.pid_torque_p_i.1)
            .await;
        let _ = self
            .set_velocity_pid(cfg.pid_velocity_p_i.0, cfg.pid_velocity_p_i.1)
            .await;
        let _ = self
            .set_position_pid(cfg.pid_position_p_i.0, cfg.pid_position_p_i.1)
            .await;

        // Set PWM modes
        let _ = self
            .write_register(
                PwmBbmHBbmL::default()
                    .with_pwm_bbm_l(cfg.pwm_bbm_l)
                    .with_pwm_bbm_h(cfg.pwm_bbm_h),
            )
            .await;
        let _ = self
            .write_register(
                PwmSvChop::default()
                    .with_pwm_sv(cfg.pwm_sv)
                    .with_pwm_chop(7),
            )
            .await;
        // Set motor parameters
        let _ = self
            .write_register(
                MotorTypeNPolePairs::default()
                    .with_motor_type(cfg.motor_type)
                    .with_n_pole_pairs(cfg.n_pole_pairs),
            )
            .await;
        // Set ADC input config
        let _ = self
            .write_register(
                AdcISelect::default()
                    .with_adc_i_ux_select(cfg.adc_i_ux_select)
                    .with_adc_i_v_select(cfg.adc_i_v_select)
                    .with_adc_i_wy_select(cfg.adc_i_wy_select)
                    .with_adc_i0_select(cfg.adc_i0_select)
                    .with_adc_i1_select(cfg.adc_i1_select),
            )
            .await;
        // Set encoder and hall parameters
        let _ = self
            .write_register(
                AencDecoderMode::default()
                    .with_aenc_deg(cfg.aenc_deg)
                    .with_aenc_dir(cfg.aenc_dir),
            )
            .await;
        let _ = self
            .write_register(AencDecoderPpr::default().with_aenc_ppr(cfg.aenc_ppr))
            .await;
        let _ = self
            .write_register(
                AbnDecoderMode::default()
                    .with_abn_apol(cfg.abn_apol)
                    .with_abn_bpol(cfg.abn_bpol)
                    .with_abn_npol(cfg.abn_npol)
                    .with_abn_use_abn_as_n(cfg.abn_use_abn_as_n)
                    .with_abn_cln(cfg.abn_cln)
                    .with_abn_direction(cfg.abn_direction),
            )
            .await;
        let _ = self
            .write_register(AbnDecoderPpr::default().with_abn_decoder_ppr(cfg.abn_decoder_ppr))
            .await;
        let _ = self
            .write_register(
                HallMode::default()
                    .with_hall_interp(cfg.hall_interp)
                    .with_hall_sync(cfg.hall_sync)
                    .with_hall_polarity(cfg.hall_polarity)
                    .with_hall_dir(cfg.hall_dir)
                    .with_hall_blank(cfg.hall_blank),
            )
            .await;
        let _ = self
            .write_register(HallDPhiMax::default().with_hall_dphi_max(cfg.hall_dphi_max))
            .await;
        let _ = self
            .write_register(
                HallPhiEPhiMOffset::default().with_hall_phi_e_offset(cfg.hall_phi_e_offset),
            )
            .await;
        // Set encoder selection
        let _ = self
            .write_register(PhiESelection::default().with_phi_e_selection(cfg.phi_e_selection))
            .await;
        let _ = self
            .write_register(
                PositionSelection::default().with_position_selection(cfg.position_selection),
            )
            .await;
        let _ = self
            .write_register(
                VelocitySelection::default()
                    .with_velocity_selection(cfg.velocity_selection)
                    .with_velocity_meter_selection(cfg.velocity_meter_selection),
            )
            .await;
        // Set advanced PID modes and limits
        let _ = self
            .write_register(
                ModeRampModeMotion::default()
                    .with_mode_pid_smpl(cfg.mode_pid_smpl)
                    .with_mode_pid_type(cfg.mode_pid_type),
            )
            .await;
        let _ = self
            .write_register(
                PidoutUqUdLimits::default().with_pidout_uq_ud_limits(cfg.pidout_uq_ud_limits),
            )
            .await;
        let _ = self
            .write_register(
                PidPositionLimitLow::default()
                    .with_pid_position_limit_low(cfg.pid_position_limit_low),
            )
            .await;
        let _ = self
            .write_register(
                PidPositionLimitHigh::default()
                    .with_pid_position_limit_high(cfg.pid_position_limit_high),
            )
            .await;
        let _ = self
            .write_register(
                PidVelocityLimit::default().with_pid_velocity_limit(cfg.pid_velocity_limit),
            )
            .await;

        // Set the PWM frequency
        let pwmfreq = self.set_pwm_freq(cfg.pwm_freq_target).await?;

        // Calibrate the ADCs
        self.calibrate_adc().await?;

        // Set the run current
        self.set_run_current().await?;

        // Set default velocity filter
        self.set_filter(
            Config::ConfigBiquadVEnable,
            1550.0,
            pwmfreq / (cfg.mode_pid_smpl + 1) as f32,
        )
        .await?;

        // Remove current offsets
        self.write_register(PidTorqueFluxOffset::default()).await?;

        self.align().await?;
        Ok(())
    }

    pub async fn dump_pid_errors(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        let _ = self
            .write_register(
                PidErrorAddr::default().with_pid_error_addr(PidError::PidErrorPidFluxError),
            )
            .await;
        let flux = self
            .read_register::<PidErrorPidFluxError>()
            .await?
            .read_pid_flux_error();
        let _ = self
            .write_register(
                PidErrorAddr::default().with_pid_error_addr(PidError::PidErrorPidTorqueError),
            )
            .await;
        let torque = self
            .read_register::<PidErrorPidTorqueError>()
            .await?
            .read_pid_torque_error();
        let _ = self
            .write_register(
                PidErrorAddr::default().with_pid_error_addr(PidError::PidErrorPidVelocityError),
            )
            .await;
        let velocity = self
            .read_register::<PidErrorPidVelocityError>()
            .await?
            .read_pid_velocity_error();
        let _ = self
            .write_register(
                PidErrorAddr::default().with_pid_error_addr(PidError::PidErrorPidPositionError),
            )
            .await;
        let position = self
            .read_register::<PidErrorPidPositionError>()
            .await?
            .read_pid_position_error();
        info!(
            "TMC PID Errors: Flux {}, Torque {}, Velocity {}, Position {}",
            flux, torque, velocity, position
        );
        Ok(())
    }

    pub async fn align(&mut self) -> Result<(), FaultDetectionError<I::BusError>> {
        let _ = self.enable_pin.set_low();
        // Save settings
        let old_mode = self
            .read_register::<ModeRampModeMotion>()
            .await?
            .read_mode_motion();
        let old_phi_e_selection = self
            .read_register::<PhiESelection>()
            .await?
            .read_phi_e_selection();
        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::StoppedMode))
            .await;
        let limit_cur = self
            .read_register::<PidTorqueFluxLimits>()
            .await?
            .read_pid_torque_flux_limits();
        let old_flux_offset = self
            .read_register::<PidTorqueFluxOffset>()
            .await?
            .read_pid_flux_offset();

        // Zero the position and encoder offsets
        let _ = self.write_register(AbnDecoderCount::default()).await;
        let _ = self
            .write_register(PidPositionActual::default().with_pid_position_actual(0))
            .await;
        let _ = self
            .write_register(
                AbnDecoderPhiEPhiMOffset::default()
                    .with_abn_decoder_phi_e_offset(0)
                    .with_abn_decoder_phi_m_offset(0),
            )
            .await;

        // Set constant zero phi_e and temporarily disable encoder
        let _ = self
            .write_register(PhiEExt::default().with_phi_e_ext(0))
            .await;
        let _ = self
            .write_register(PhiESelection::default().with_phi_e_selection(1))
            .await;

        // Start at a low voltage and see if we detect current
        // This should wind up being about 0.1 V
        let test_u = VM_RANGE / (4 * self.voltage_scale_i);
        info!("TMC Alignment test_U: {}", test_u);
        // Still in stopped mode
        let _ = self
            .write_register(UqUdExt::default().with_ud_ext(test_u as i16))
            .await;
        let _ = self
            .write_register(PidPositionTarget::default().with_pid_position_target(0))
            .await;
        let _ = self
            .write_register(PidVelocityTarget::default().with_pid_velocity_target(0))
            .await;
        let _ = self
            .write_register(
                PidTorqueFluxTarget::default()
                    .with_pid_torque_target(0)
                    .with_pid_flux_target(0),
            )
            .await;

        let _ = self
            .write_register(PidPositionActual::default().with_pid_position_actual(0))
            .await;
        let reg = self.read_register::<PwmSvChop>().await?;
        let _ = self.write_register(reg.with_pwm_chop(7)).await;
        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::UqUdExtMode))
            .await;

        // Enable the power stage and check the current
        let _ = self.enable_pin.set_high();
        Timer::after(Duration::from_millis(100)).await;
        let (iux, iwy, iv) = self.get_adc_currents().await.unwrap_or((0, 0, 0));
        let _ = self.write_register(reg.with_pwm_chop(0)).await;
        info!(
            "TMC Alignment Currents: Iux {}, Iwy {}, Iv {}",
            iux, iwy, iv
        );
        let _ = self.enable_pin.set_low();

        // Ok, we have a current, calculate an approx resistance and new test voltage
        let i = (iux.abs() + iwy.abs() + iv.abs()) as u16;
        let f = self.calculate_flux_limit(self.run_current * 0.5);
        let test2_u: u16 = (test_u as u32 * f as u32 / i as u32) as u16;
        info!("TMC Alignment test2_U: {}", test2_u);
        // let _ = self
        //     .write_register(UqUdExt::default().with_ud_ext(test2_u as i16))
        //     .await;

        let _ = self
            .write_register(
                PidTorqueFluxTarget::default()
                    .with_pid_torque_target(0)
                    .with_pid_flux_target(f as i16),
            )
            .await;

        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::TorqueMode))
            .await;

        // Now pulse the voltage and get the rotor aligned
        let posreg = self.read_register::<PidPositionActual>().await?;
        let mut last_pos = posreg.read_pid_position_actual();
        let _ = self.write_register(reg.with_pwm_chop(7)).await;
        let _ = self.enable_pin.set_high();
        let mut c = 0;
        for _ in 0..60 {
            Timer::after(Duration::from_millis(2)).await;
            let (iux, iwy, iv) = self.get_adc_currents().await.unwrap_or((0, 0, 0));
            info!(
                "TMC Alignment Currents: Iux {}, Iwy {}, Iv {}",
                iux, iwy, iv
            );
            let reg = self.read_register::<PidPositionActual>().await?;
            let pos = reg.read_pid_position_actual();
            info!("  Position Actual: {}", pos);
            if pos != last_pos || c < 20 {
                last_pos = pos;
                c += 1;
            } else {
                // No change, we are aligned
                break;
            }
        }

        // We are now mechanically aligned, set up the encoder offsets.
        let _ = self.write_register(AbnDecoderCount::default()).await;
        let _ = self
            .write_register(PidPositionActual::default().with_pid_position_actual(0))
            .await;
        let _ = self
            .write_register(
                AbnDecoderPhiEPhiMOffset::default()
                    .with_abn_decoder_phi_e_offset(0)
                    .with_abn_decoder_phi_m_offset(0),
            )
            .await;
        Timer::after(Duration::from_millis(200)).await;

        // Switch back to normal operation
        let _ = self.write_register(reg.with_pwm_chop(0)).await;
        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(MotionMode::StoppedMode))
            .await;
        // let _ = self.enable_pin.set_low();
        // let _ = self.write_register(UqUdExt::default().with_ud_ext(0)).await;
        let _ = self
            .write_register(
                PidTorqueFluxTarget::default()
                    .with_pid_torque_target(0)
                    .with_pid_flux_target(0),
            )
            .await;
        let _ = self
            .write_register(PhiESelection::default().with_phi_e_selection(old_phi_e_selection))
            .await;
        let _ = self
            .write_register(PidTorqueFluxLimits::default().with_pid_torque_flux_limits(limit_cur))
            .await;
        let _ = self
            .write_register(PidTorqueFluxOffset::default().with_pid_flux_offset(old_flux_offset))
            .await;

        // Set the run current
        let _ = self.set_run_current().await?;
        Timer::after(Duration::from_millis(200)).await;

        let _ = self
            .write_register(ModeRampModeMotion::default().with_mode_motion(old_mode))
            .await;
        Ok(())
    }

    // Run device. Never returns.
    pub async fn run(&mut self) -> ! {
        let mut ticker = TMCTimeIterator::new();
        let mut torque_offset: i32 = 0;

        loop {
            // torque_offset /= 2;
            // self.write_register(
            //     PidTorqueFluxOffset::default().with_pid_torque_offset(torque_offset as i16),
            // )
            // .await
            // .ok();
            let pos_actual = self.get_pid_position_actual().await.unwrap();
            let target_actual = self
                .read_register::<PidPositionTarget>()
                .await
                .unwrap()
                .read_pid_position_target();
            let v_actual = self
                .read_register::<PidVelocityActual>()
                .await
                .unwrap()
                .read_pid_velocity_actual();
            let v_target = self
                .read_register::<PidVelocityTarget>()
                .await
                .unwrap()
                .read_pid_velocity_target();
            let r = self.read_register::<PidTorqueFluxActual>().await.unwrap();
            let torque_actual = r.read_pid_torque_actual();
            let flux_actual = r.read_pid_flux_actual();
            info!(
                "TMC Move to {} from actual {} target {} v_actual {} v_target {} torque {} flux {}",
                self.last_pos,
                pos_actual,
                target_actual,
                v_actual,
                v_target,
                torque_actual,
                flux_actual
            );
            self.dump_pid_errors().await.ok();
            let (iux, iwy, iv) = self.get_adc_currents().await.unwrap_or((0, 0, 0));
            info!("TMC Currents: Iux {}, Iwy {}, Iv {}", iux, iwy, iv);
            // let (i0, i1) = self.get_raw_adc_currents().await.unwrap_or((0, 0));
            // info!("TMC Raw Currents: I0 {}, I1 {}", i0, i1);
            while let Some(cmd) = self.command_rx.try_receive().ok() {
                match cmd {
                    TMCCommand::Enable => {
                        info!("TMC Command {}", cmd);
                        self.enable_motor().await.ok();
                        self.set_motion_mode(MotionMode::PositionMode).await.ok();
                    }
                    TMCCommand::Disable => {
                        info!("TMC Command {}", cmd);
                        self.disable_motor().await.ok();
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
                    TMCCommand::Move(pos, vel, _accel) => {
                        if self.last_pos != pos {
                            info!("TMC Command {}", cmd);

                            // Simple velocity feedforward
                            // torque_offset = (1e-4 * vel * self.current_limit as f32) as i32;
                            if torque_offset > self.current_limit as i32 {
                                torque_offset = self.current_limit as i32;
                            }
                            if torque_offset < -(self.current_limit as i32) {
                                torque_offset = -(self.current_limit as i32);
                            }
                        }
                        self.last_pos = pos;
                        if self.enabled {
                            self.write_register(
                                PidPositionTarget::default().with_pid_position_target(pos),
                            )
                            .await
                            .ok();
                            // self.write_register(
                            //     PidTorqueFluxOffset::default()
                            //         .with_pid_torque_offset(torque_offset as i16),
                            // )
                            // .await
                            // .ok();
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
