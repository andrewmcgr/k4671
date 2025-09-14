use const_builder::ConstBuilder;

#[derive(ConstBuilder)]
#[builder(default)]
pub struct TMC4671Config {
    #[builder(default = 1.155)]
    pub current_scale_ma_lsb: f32,
    #[builder(default = 0.5)]
    pub run_current: f32,
    #[builder(default = 0.0)]
    pub flux_current: f32,
    #[builder(default = 43.64)]
    pub voltage_scale: f32,
    #[builder(default = 50e3)]
    pub pwm_freq_target: f32,
    #[builder(default = 50)]
    pub n_pole_pairs: u16,
    #[builder(default = 10)]
    pub pwm_bbm_l: u8,
    #[builder(default = 10)]
    pub pwm_bbm_h: u8,
    #[builder(default = false)]
    pub pwm_sv: bool,
    #[builder(default = 2)]
    pub motor_type: u8,
    #[builder(default = 0)]
    pub adc_i_ux_select: u8,
    #[builder(default = 2)]
    pub adc_i_v_select: u8,
    #[builder(default = 1)]
    pub adc_i_wy_select: u8,
    #[builder(default = 0)]
    pub adc_i0_select: u8,
    #[builder(default = 1)]
    pub adc_i1_select: u8,
    #[builder(default = true)]
    pub aenc_deg: bool,
    #[builder(default = false)]
    pub aenc_dir: bool,
    #[builder(default = 1)]
    pub aenc_ppr: u16,
    #[builder(default = false)]
    pub abn_apol: bool,
    #[builder(default = false)]
    pub abn_bpol: bool,
    #[builder(default = false)]
    pub abn_npol: bool,
    #[builder(default = false)]
    pub abn_use_abn_as_n: bool,
    #[builder(default = false)]
    pub abn_cln: bool,
    #[builder(default = false)]
    pub abn_direction: bool,
    #[builder(default = 4000)]
    pub abn_decoder_ppr: u32,
    #[builder(default = false)]
    pub hall_interp: bool,
    #[builder(default = true)]
    pub hall_sync: bool,
    #[builder(default = false)]
    pub hall_polarity: bool,
    #[builder(default = false)]
    pub hall_dir: bool,
    #[builder(default = 0xAAAA)]
    pub hall_dphi_max: u32,
    #[builder(default = 0)]
    pub hall_phi_e_offset: i16,
    #[builder(default = 2)]
    pub hall_blank: u16,
    #[builder(default = 3)]
    pub phi_e_selection: u32,
    #[builder(default = 9)]
    pub position_selection: u32,
    #[builder(default = 9)]
    pub velocity_selection: u8,
    #[builder(default = true)] // PWM frequency velocity meter
    pub velocity_meter_selection: bool,
    #[builder(default = 0)] // Advanced PID samples position at fPWM
    pub mode_pid_smpl: u8,
    #[builder(default = true)] // Advanced PID mode
    pub mode_pid_type: bool,
    #[builder(default = 31500)] // Voltage limit, 32768 = Vm
    pub pidout_uq_ud_limits: u16,
    #[builder(default=-0x10000000)]
    pub pid_position_limit_low: i32,
    #[builder(default = 0x10000000)]
    pub pid_position_limit_high: i32,
    #[builder(default = 0x10000000)]
    pub pid_velocity_limit: u32,
    #[builder(default = (2.82, 0.00277))]
    pub pid_position_p_i: (f32, f32),
    #[builder(default = (1.408, 0.00826))]
    pub pid_velocity_p_i: (f32, f32),
    #[builder(default = (4.879, 0.0571))]
    pub pid_torque_p_i: (f32, f32),
    #[builder(default = (4.879, 0.0571))]
    pub pid_flux_p_i: (f32, f32),
}
