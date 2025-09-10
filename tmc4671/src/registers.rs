use embedded_interfaces::codegen::interface_objects;

pub type TMC4671I2cCodec =
    embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
pub type TMC4671SpiCodec =
    embedded_interfaces::registers::spi::codecs::standard_codec::StandardCodec<
        1,
        6,
        0,
        7,
        false,
        0,
    >;

pub const DEVICE_ID_VALID: u32 = 0x34363731;

interface_objects! {
  register_defaults {
      codec_error = (),
      i2c_codec = TMC4671I2cCodec,
      spi_codec = TMC4671SpiCodec,
  }

  register_devices [ super::TMC4671 ]

  register ChipinfoData(addr = 0x0, mode = rw, size=4) {
    chipinfo_data: u32,
  }

  enum Chipinfo: u32{32} {
    0 ChipinfoSiType,
    1 ChipinfoSiVersion,
    2 ChipinfoSiDate,
    3 ChipinfoSiTime,
    4 ChipinfoSiVariant,
    5 ChipinfoSiBuild,
    _ Invalid(u32),
  }

  register ChipinfoAddr(addr = 0x1, mode = rw, size=4) {
    chipinfo_addr: Chipinfo = Chipinfo::ChipinfoSiType,
  }

  register ChipinfoSiType(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_type: u32,
  }

  register ChipinfoSiVersion(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_version_hi: u16,
    chipinfo_si_version_lo: u16,
  }

  register ChipinfoSiDate(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_date: u32,
  }

  register ChipinfoSiTime(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_time: u32,
  }

  register ChipinfoSiVariant(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_variant: u32,
  }

  register ChipinfoSiBuild(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_build: u32,
  }

  register AdcRawData(addr = 0x2, mode = rw, size=4) {
    adc_raw_data: u32,
  }

  enum AdcRaw: u32{32} {
    0 AdcI1RawAdcI0Raw,
    1 AdcAgpiARawAdcVmRaw,
    2 AdcAencUxRawAdcAgpiBRaw,
    3 AdcAencWyRawAdcAencVnRaw,
    _ Invalid(u32),
  }

  register AdcRawAddr(addr = 0x3, mode = rw, size=4) {
    adc_raw_addr: AdcRaw = AdcRaw::AdcI1RawAdcI0Raw,
  }

  register AdcI1RawAdcI0Raw(addr = 0x2, mode = rw, size=4) {
    adc_i0_raw: u16[0..16],
    adc_i1_raw: u16[16..32],
  }

  register AdcAgpiARawAdcVmRaw(addr = 0x2, mode = rw, size=4) {
    adc_agpi_a_raw: u16[0..16],
    adc_vm_raw: u16[16..32],
  }

  register AdcAencUxRawAdcAgpiBRaw(addr = 0x2, mode = rw, size=4) {
    adc_aenc_ux_raw: u16[0..16],
    adc_agpi_b_raw: u16[16..32],
  }

  register AdcAencWyRawAdcAencVnRaw(addr = 0x2, mode = rw, size=4) {
    adc_aenc_wy_raw: u16[0..16],
    adc_aenc_vn_raw: u16[16..32],
  }

  register DsadcMcfgBMcfgA(addr = 0x4, mode = rw, size=4) {
    cfg_dsmodulator_a: u8[0..2],
    mclk_polarity_a: bool[2],
    mdat_polarity_a: bool[3],
    sel_nclk_mclk_ia: bool[4],
    _: u16[5..16],
    cfg_dsmodulator_b: u8[16..18],
    mclk_polarity_b: bool[18],
    mdat_polarity_b: bool[19],
    sel_nclk_mclk_ib: bool[20],
    _: u16[21..32],
  }

  register DsadcMclkA(addr = 0x5, mode = rw, size=4) {
    dsadc_mclk_a: u32,
  }

  register DsadcMclkB(addr = 0x6, mode = rw, size=4) {
    dsadc_mclk_b: u32,
  }

  register DsadcMdecBMdecA(addr = 0x7, mode = rw, size=4) {
    dsadc_mdec_a: u16[0..16],
    dsadc_mdec_b: u16[16..32],
  }

  register AdcI1ScaleOffset(addr = 0x8, mode = rw, size=4) {
    adc_i1_offset: u16[0..16],
    adc_i1_scale: u16[16..32],
  }

  register AdcI0ScaleOffset(addr = 0x9, mode = rw, size=4) {
    adc_i0_offset: u16[0..16],
    adc_i0_scale: u16[16..32],
  }

  register AdcISelect(addr = 0xa, mode = rw, size=4) {
    adc_i0_select: u8[0..8],
    adc_i1_select: u8[8..16],
    _: u8[16..24],
    adc_i_ux_select: u8[24..26],
    adc_iv_select: u8[26..28],
    adc_i_wy_select: u8[28..30],
    _: u8[30..32],
  }

  register AdcI1I0Ext(addr = 0xb, mode = rw, size=4) {
    adc_i0_ext: u16[0..16],
    adc_i1_ext: u16[16..32],
  }

  register DsAnalogInputStageCfg(addr = 0xc, mode = rw, size=4) {
    cfg_adc_i0: u8[0..4],
    cfg_adc_i1: u8[4..8],
    cfg_adc_vm: u8[8..12],
    cfg_adc_agpi_a: u8[12..16],
    cfg_adc_agpi_b: u8[16..20],
    cfg_adc_aenc_ux: u8[20..24],
    cfg_adc_aenc_vn: u8[24..28],
    cfg_adc_aenc_wy: u8[28..32],
  }

  register Aenc0ScaleOffset(addr = 0xd, mode = rw, size=4) {
    aenc0_offset: u16[0..16],
    aenc0_scale: u16[16..32],
  }

  register Aenc1ScaleOffset(addr = 0xe, mode = rw, size=4) {
    aenc1_offset: u16[0..16],
    aenc1_scale: u16[16..32],
  }

  register Aenc2ScaleOffset(addr = 0xf, mode = rw, size=4) {
    aenc2_offset: u16[0..16],
    aenc2_scale: u16[16..32],
  }

  register AencSelect(addr = 0x11, mode = rw, size=4) {
    aenc0_select: u8[0..8],
    aenc1_select: u8[8..16],
    aenc2_select: u8[16..24],
    _: u8[24..32],
  }

  register AdcIwyIux(addr = 0x12, mode = rw, size=4) {
    adc_iux: u16[0..16],
    adc_iwy: u16[16..32],
  }

  register AdcIv(addr = 0x13, mode = rw, size=4) {
    adc_iv: u16[0..16],
    _: u16[16..32],
  }

  register AencWyUx(addr = 0x15, mode = rw, size=4) {
    aenc_ux: u16[0..16],
    aenc_wy: u16[16..32],
  }

  register AencVn(addr = 0x16, mode = rw, size=4) {
    aenc_vn: u16[0..16],
    _: u16[16..32],
  }

  register PwmPolarities(addr = 0x17, mode = rw, size=4) {
    pwm_polarities0: bool[0],
    pwm_polarities1: bool[1],
    _: u32[2..32],
  }

  register PwmMaxcnt(addr = 0x18, mode = rw, size=4) {
    pwm_maxcnt: u32,
  }

  register PwmBbmHBbmL(addr = 0x19, mode = rw, size=4) {
    pwm_bbm_l: u8[0..8],
    pwm_bbm_h: u8[8..16],
    _: u16[16..32],
  }

  register PwmSvChop(addr = 0x1a, mode = rw, size=4) {
    pwm_chop: u8[0..8],
    pwm_sv: bool[8],
    _: u32[9..32],
  }

  register MotorTypeNPolePairs(addr = 0x1b, mode = rw, size=4) {
    n_pole_pairs: u16[0..16],
    motor_type: u8[16..24],
    _: u8[24..32],
  }

  register PhiEExt(addr = 0x1c, mode = rw, size=4) {
    phi_e_ext: u32,
  }

  register OpenloopMode(addr = 0x1f, mode = rw, size=4) {
    openloop_phi_direction: bool[12],
    _: u32[0..12,13..32],
  }

  register OpenloopAcceleration(addr = 0x20, mode = rw, size=4) {
    openloop_acceleration: u32,
  }

  register OpenloopVelocityTarget(addr = 0x21, mode = rw, size=4) {
    openloop_velocity_target: i32,
  }

  register OpenloopVelocityActual(addr = 0x22, mode = rw, size=4) {
    openloop_velocity_actual: i32,
  }

  register OpenloopPhi(addr = 0x23, mode = rw, size=4) {
    openloop_phi: i32,
  }

  register UqUdExt(addr = 0x24, mode = rw, size=4) {
    ud_ext: i16[0..16],
    uq_ext: i16[16..32],
  }

  register AbnDecoderMode(addr = 0x25, mode = rw, size=4) {
    abn_apol: bool[0],
    abn_bpol: bool[1],
    abn_npol: bool[2],
    abn_use_abn_as_n: bool[3],
    abn_cln: bool[8],
    abn_direction: bool[12],
    _: u32[4..8,9..12,13..32],
  }

  register AbnDecoderPpr(addr = 0x26, mode = rw, size=4) {
    abn_decoder_ppr: u32,
  }

  register AbnDecoderCount(addr = 0x27, mode = rw, size=4) {
    abn_decoder_count: u32,
  }

  register AbnDecoderCountN(addr = 0x28, mode = rw, size=4) {
    abn_decoder_count_n: u32,
  }

  register AbnDecoderPhiEPhiMOffset(addr = 0x29, mode = rw, size=4) {
    abn_decoder_phi_m_offset: u16[0..16],
    abn_decoder_phi_e_offset: u16[16..32],
  }

  register AbnDecoderPhiEPhiM(addr = 0x2a, mode = rw, size=4) {
    abn_decoder_phi_m: u16[0..16],
    abn_decoder_phi_e: u16[16..32],
  }

  register Abn2DecoderMode(addr = 0x2c, mode = rw, size=4) {
    abn2_apol: bool[0],
    abn2_bpol: bool[1],
    abn2_npol: bool[2],
    abn2_use_abn_as_n: bool[3],
    abn2_cln: bool[8],
    abn2_direction: bool[12],
    _: u32[4..8,9..12,13..32],
  }

  register Abn2DecoderPpr(addr = 0x2d, mode = rw, size=4) {
    abn2_decoder_ppr: u32,
  }

  register Abn2DecoderCount(addr = 0x2e, mode = rw, size=4) {
    abn2_decoder_count: u32,
  }

  register Abn2DecoderCountN(addr = 0x2f, mode = rw, size=4) {
    abn2_decoder_count_n: u32,
  }

  register Abn2DecoderPhiMOffset(addr = 0x30, mode = rw, size=4) {
    abn2_decoder_phi_m_offset: u32,
  }

  register Abn2DecoderPhiM(addr = 0x31, mode = rw, size=4) {
    abn2_decoder_phi_m: u32,
  }

  register HallMode(addr = 0x33, mode = rw, size=4) {
    hall_polarity: bool[0],
    hall_sync: bool[4],
    hall_interp: bool[8],
    hall_dir: bool[12],
    hall_blank: u16[16..28],
    _: u32[1,2,3,5..8,9..12,13..16,28..32],
  }

  register HallPosition060000(addr = 0x34, mode = rw, size=4) {
    hall_position000: u16[0..16],
    hall_position060: u16[16..32],
  }

  register HallPosition180120(addr = 0x35, mode = rw, size=4) {
    hall_position120: u16[0..16],
    hall_position180: u16[16..32],
  }

  register HallPosition300240(addr = 0x36, mode = rw, size=4) {
    hall_position240: u16[0..16],
    hall_position300: u16[16..32],
  }

  register HallPhiEPhiMOffset(addr = 0x37, mode = rw, size=4) {
    hall_phi_m_offset: u16[0..16],
    hall_phi_e_offset: u16[16..32],
  }

  register HallDphiMax(addr = 0x38, mode = rw, size=4) {
    hall_dphi_max: u32,
  }

  register HallPhiEInterpolatedPhiE(addr = 0x39, mode = rw, size=4) {
    hall_phi_e: u16[0..16],
    hall_phi_e_interpolated: u16[16..32],
  }

  register HallPhiM(addr = 0x3a, mode = rw, size=4) {
    hall_phi_m: u32,
  }

  register AencDecoderMode(addr = 0x3b, mode = rw, size=4) {
    aenc_deg: bool[0],
    aenc_dir: bool[12],
    _: u32[1..12,13..32],
  }

  register AencDecoderNThreshold(addr = 0x3c, mode = rw, size=4) {
    aenc_decoder_n_threshold: u32,
  }

  register AencDecoderPhiARaw(addr = 0x3d, mode = rw, size=4) {
    aenc_decoder_phi_a_raw: u32,
  }

  register AencDecoderPhiAOffset(addr = 0x3e, mode = rw, size=4) {
    aenc_decoder_phi_a_offset: u32,
  }

  register AencDecoderPhiA(addr = 0x3f, mode = rw, size=4) {
    aenc_decoder_phi_a: u32,
  }

  register AencDecoderPpr(addr = 0x40, mode = rw, size=4) {
    aenc_ppr: u16[0..16],
    _: u16[16..32],
  }

  register AencDecoderCount(addr = 0x41, mode = rw, size=4) {
    aenc_decoder_count: u32,
  }

  register AencDecoderCountN(addr = 0x42, mode = rw, size=4) {
    aenc_decoder_count_n: u32,
  }

  register AencDecoderPhiEPhiMOffset(addr = 0x45, mode = rw, size=4) {
    aenc_decoder_phi_m_offset: u16[0..16],
    aenc_decoder_phi_e_offset: u16[16..32],
  }

  register AencDecoderPhiEPhiM(addr = 0x46, mode = rw, size=4) {
    aenc_decoder_phi_m: u16[0..16],
    aenc_decoder_phi_e: u16[16..32],
  }

  register ConfigData(addr = 0x4d, mode = rw, size=4) {
    config_data: u32,
  }

  enum Config: u32{32} {
    1 ConfigBiquadXA1,
    2 ConfigBiquadXA2,
    4 ConfigBiquadXB0,
    5 ConfigBiquadXB1,
    6 ConfigBiquadXB2,
    7 ConfigBiquadXEnable,
    9 ConfigBiquadVA1,
    10 ConfigBiquadVA2,
    12 ConfigBiquadVB0,
    13 ConfigBiquadVB1,
    14 ConfigBiquadVB2,
    15 ConfigBiquadVEnable,
    17 ConfigBiquadTA1,
    18 ConfigBiquadTA2,
    20 ConfigBiquadTB0,
    21 ConfigBiquadTB1,
    22 ConfigBiquadTB2,
    23 ConfigBiquadTEnable,
    25 ConfigBiquadFA1,
    26 ConfigBiquadFA2,
    28 ConfigBiquadFB0,
    29 ConfigBiquadFB1,
    30 ConfigBiquadFB2,
    31 ConfigBiquadFEnable,
    40 FeedForwardVelocityGain,
    41 FeedForwardVelocityFilterConstant,
    42 FeedForwardTorqueGain,
    43 FeedForwardTorqueFilterConstant,
    51 ConfigRefSwitchConfig,
    60 ConfigSinglePinIfStatusCfg,
    61 ConfigSinglePinIfScaleOffset,
    62 ConfigAdvancedPiRepresent,
    _ Invalid(u32),
  }

  register ConfigAddr(addr = 0x4e, mode = rw, size=4) {
    config_addr: Config = Config::ConfigBiquadXA1,
  }

  register ConfigBiquadXA1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_xa1: u32,
  }

  register ConfigBiquadXA2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_xa2: u32,
  }

  register ConfigBiquadXB0(addr = 0x4d, mode = rw, size=4) {
    config_biquad_xb0: u32,
  }
  register ConfigBiquadXB1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_xb1: u32,
  }
  register ConfigBiquadXB2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_xb2: u32,
  }
  register ConfigBiquadXEnable(addr = 0x4d, mode = rw, size=4) {
    config_biquad_x_enable: u32,
  }
  register ConfigBiquadVA1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_va1: u32,
  }
  register ConfigBiquadVA2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_va2: u32,
  }
  register ConfigBiquadVB0(addr = 0x4d, mode = rw, size=4) {
    config_biquad_vb0: u32,
  }
  register ConfigBiquadVB1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_vb1: u32,
  }
  register ConfigBiquadVB2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_vb2: u32,
  }
  register ConfigBiquadVEnable(addr = 0x4d, mode = rw, size=4) {
    config_biquad_v_enable: u32,
  }
  register ConfigBiquadTA1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_ta1: u32,
  }
  register ConfigBiquadTA2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_ta2: u32,
  }
  register ConfigBiquadTB0(addr = 0x4d, mode = rw, size=4) {
    config_biquad_tb0: u32,
  }
  register ConfigBiquadTB1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_tb1: u32,
  }
  register ConfigBiquadTB2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_tb2: u32,
  }
  register ConfigBiquadTEnable(addr = 0x4d, mode = rw, size=4) {
    config_biquad_t_enable: u32,
  }
  register ConfigBiquadFA1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_fa1: u32,
  }
  register ConfigBiquadFA2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_fa2: u32,
  }
  register ConfigBiquadFB0(addr = 0x4d, mode = rw, size=4) {
    config_biquad_fb0: u32,
  }
  register ConfigBiquadFB1(addr = 0x4d, mode = rw, size=4) {
    config_biquad_fb1: u32,
  }
  register ConfigBiquadFB2(addr = 0x4d, mode = rw, size=4) {
    config_biquad_fb2: u32,
  }
  register ConfigBiquadFEnable(addr = 0x4d, mode = rw, size=4) {
    config_biquad_f_enable: u32,
  }
  register FeedForwardVelocityGain(addr = 0x4d, mode = rw, size=4) {
    feed_forward_velocity_gain: u32,
  }
  register FeedForwardVelocityFilterConstant(addr = 0x4d, mode = rw, size=4) {
    feed_forward_velocity_filter_constant: u32,
  }
  register FeedForwardTorqueGain(addr = 0x4d, mode = rw, size=4) {
    feed_forward_torque_gain: u32,
  }
  register FeedForwardTorqueFilterConstant(addr = 0x4d, mode = rw, size=4) {
    feed_forward_torque_filter_constant: u32,
  }
  register ConfigRefSwitchConfig(addr = 0x4d, mode = rw, size=4) {
    config_ref_switch_config: u32,
  }
  register ConfigSinglePinIfStatusCfg(addr = 0x4d, mode = rw, size=4) {
    config_single_pin_if_status_cfg: u32,
  }
  register ConfigSinglePinIfScaleOffset(addr = 0x4d, mode = rw, size=4) {
    config_single_pin_if_scale_offset: u32,
  }

  register VelocitySelection(addr = 0x50, mode = rw, size=4) {
    velocity_mode_selection: u8[0..8],
    velocity_meter_selection: u8[8..16],
    _: u16[16..32],
  }

  register PositionSelection(addr = 0x51, mode = rw, size=4) {
    position_selection: u32,
  }
  register PhiESelection(addr = 0x52, mode = rw, size=4) {
    phi_e_selection: u32,
  }
  register PhiE(addr = 0x53, mode = rw, size=4) {
    phi_e: i16,
    _: u16[16..32],
  }

  register PidFluxPFluxI(addr = 0x54, mode = rw, size=4) {
    pid_flux_i: i16[0..16],
    pid_flux_p: i16[16..32],
  }

  register PidTorquePTorqueI(addr = 0x56, mode = rw, size=4) {
    pid_torque_i: i16[0..16],
    pid_torque_p: i16[16..32],
  }

  register PidVelocityPVelocityI(addr = 0x58, mode = rw, size=4) {
    pid_velocity_i: i16[0..16],
    pid_velocity_p: i16[16..32],
  }

  register PidPositionPPositionI(addr = 0x5a, mode = rw, size=4) {
    pid_position_i: i16[0..16],
    pid_position_p: i16[16..32],
  }

  register PidoutUqUdLimits(addr = 0x5d, mode = rw, size=4) {
    pidout_uq_ud_limits: u16,
    _: u16[16..32],
  }
  register PidTorqueFluxLimits(addr = 0x5e, mode = rw, size=4) {
    pid_torque_flux_limits: u16,
    _: u16[16..32],
  }
  register PidVelocityLimit(addr = 0x60, mode = rw, size=4) {
    pid_velocity_limit: u32,
  }
  register PidPositionLimitLow(addr = 0x61, mode = rw, size=4) {
    pid_position_limit_low: i32,
  }
  register PidPositionLimitHigh(addr = 0x62, mode = rw, size=4) {
    pid_position_limit_high: i32,
  }

  register ModeRampModeMotion(addr = 0x63, mode = rw, size=4) {
    mode_motion: u8[0..8],
    mode_pid_smpl: u8[24..31],
    mode_ff: u8[16..24],
    mode_pid_type: bool[31],
    _: u8[8..16]
  }

  register PidTorqueFluxTarget(addr = 0x64, mode = rw, size=4) {
    pid_flux_target: i16[0..16],
    pid_torque_target: i16[16..32],
  }

  register PidTorqueFluxOffset(addr = 0x65, mode = rw, size=4) {
    pid_flux_offset: i16[0..16],
    pid_torque_offset: i16[16..32],
  }

  register PidVelocityTarget(addr = 0x66, mode = rw, size=4) {
    pid_velocity_target: i32,
  }
  register PidVelocityOffset(addr = 0x67, mode = rw, size=4) {
    pid_velocity_offset: i32,
  }
  register PidPositionTarget(addr = 0x68, mode = rw, size=4) {
    pid_position_target: i32,
  }

  register PidTorqueFluxActual(addr = 0x69, mode = rw, size=4) {
    pid_flux_actual: u16[0..16],
    pid_torque_actual: u16[16..32],
  }

  register PidVelocityActual(addr = 0x6a, mode = rw, size=4) {
    pid_velocity_actual: i32,
  }
  register PidPositionActual(addr = 0x6b, mode = rw, size=4) {
    pid_position_actual: i32,
  }
  register PidErrorData(addr = 0x6c, mode = rw, size=4) {
    pid_error_data: i32,
  }

  enum PidError: u32{32} {
    0 PidErrorPidTorqueError,
    1 PidErrorPidFluxError,
    2 PidErrorPidVelocityError,
    3 PidErrorPidPositionError,
    4 PidErrorPidTorqueErrorSum,
    5 PidErrorPidFluxErrorSum,
    6 PidErrorPidVelocityErrorSum,
    7 PidErrorPidPositionErrorSum,
    _ Invalid(u32),
  }

  register PidErrorAddr(addr = 0x6d, mode = rw, size=4) {
    value: PidError = PidError::PidErrorPidTorqueError,
  }

  register PidErrorPidTorqueError(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_torque_error: u32,
  }
  register PidErrorPidFluxError(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_flux_error: u32,
  }
  register PidErrorPidVelocityError(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_velocity_error: u32,
  }
  register PidErrorPidPositionError(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_position_error: u32,
  }
  register PidErrorPidTorqueErrorSum(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_torque_error_sum: u32,
  }
  register PidErrorPidFluxErrorSum(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_flux_error_sum: u32,
  }
  register PidErrorPidVelocityErrorSum(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_velocity_error_sum: u32,
  }
  register PidErrorPidPositionErrorSum(addr = 0x6c, mode = rw, size=4) {
    pid_error_pid_position_error_sum: u32,
  }

  register InterimData(addr = 0x6e, mode = rw, size=4) {
    interim_data: u32,
  }

  enum Interim: u32{32} {
    0 InterimPidinTargetTorque,
    1 InterimPidinTargetFlux,
    2 InterimPidinTargetVelocity,
    3 InterimPidinTargetPosition,
    4 InterimPidoutTargetTorque,
    5 InterimPidoutTargetFlux,
    6 InterimPidoutTargetVelocity,
    7 InterimPidoutTargetPosition,
    8 InterimFocIwyIux,
    9 InterimFocIv,
    10 InterimFocIbIa,
    11 InterimFocIqId,
    12 InterimFocUqUd,
    13 InterimFocUqUdLimited,
    14 InterimFocUbUa,
    15 InterimFocUwyUux,
    16 InterimFocUv,
    17 InterimPwmWyUx,
    18 InterimPwmUv,
    19 InterimAdcI1I0,
    20 InterimPidTorqueTargetFluxTargetTorqueActualFluxActualDiv256,
    21 InterimPidTorqueTargetTorqueActual,
    22 InterimPidFluxTargetFluxActual,
    23 InterimPidVelocityTargetVelocityActualDiv256,
    24 InterimPidVelocityTargetVelocityActual,
    25 InterimPidPositionTargetPositionActualDiv256,
    26 InterimPidPositionTargetPositionActual,
    27 InterimFfVelocity,
    28 InterimFfTorque,
    29 InterimActualVelocityPptm,
    30 InterimRefSwitchStatus,
    31 InterimHomePosition,
    32 InterimLeftPosition,
    33 InterimRightPosition,
    42 InterimSinglePinIfPwmDutyCycleTorqueTarget,
    43 InterimSinglePinIfVelocityTarget,
    44 InterimSinglePinIfPositionTarget,
    _ Invalid(u32),
  }

  register InterimAddr(addr = 0x6f, mode = rw, size=4) {
    value: Interim = Interim::InterimPidinTargetTorque,
  }

  register InterimPidinTargetTorque(addr = 0x6e, mode = rw, size=4) {
    interim_pidin_target_torque: u32,
  }
  register InterimPidinTargetFlux(addr = 0x6e, mode = rw, size=4) {
    interim_pidin_target_flux: u32,
  }
  register InterimPidinTargetVelocity(addr = 0x6e, mode = rw, size=4) {
    interim_pidin_target_velocity: u32,
  }
  register InterimPidinTargetPosition(addr = 0x6e, mode = rw, size=4) {
    interim_pidin_target_position: u32,
  }
  register InterimPidoutTargetTorque(addr = 0x6e, mode = rw, size=4) {
    interim_pidout_target_torque: u32,
  }
  register InterimPidoutTargetFlux(addr = 0x6e, mode = rw, size=4) {
    interim_pidout_target_flux: u32,
  }
  register InterimPidoutTargetVelocity(addr = 0x6e, mode = rw, size=4) {
    interim_pidout_target_velocity: u32,
  }
  register InterimPidoutTargetPosition(addr = 0x6e, mode = rw, size=4) {
    interim_pidout_target_position: u32,
  }
  register InterimFocIwyIux(addr = 0x6e, mode = rw, size=4) {
    interim_foc_iwy_iux: u32,
  }
  register InterimFocIv(addr = 0x6e, mode = rw, size=4) {
    interim_foc_iv: u32,
  }
  register InterimFocIbIa(addr = 0x6e, mode = rw, size=4) {
    interim_foc_ib_ia: u32,
  }
  register InterimFocIqId(addr = 0x6e, mode = rw, size=4) {
    interim_foc_iq_id: u32,
  }
  register InterimFocUqUd(addr = 0x6e, mode = rw, size=4) {
    interim_foc_uq_ud: u32,
  }
  register InterimFocUqUdLimited(addr = 0x6e, mode = rw, size=4) {
    interim_foc_uq_ud_limited: u32,
  }
  register InterimFocUbUa(addr = 0x6e, mode = rw, size=4) {
    interim_foc_ub_ua: u32,
  }
  register InterimFocUwyUux(addr = 0x6e, mode = rw, size=4) {
    interim_foc_uwy_uux: u32,
  }
  register InterimFocUv(addr = 0x6e, mode = rw, size=4) {
    interim_foc_uv: u32,
  }
  register InterimPwmUv(addr = 0x6e, mode = rw, size=4) {
    interim_pwm_uv: u32,
  }
  register InterimAdcI1I0(addr = 0x6e, mode = rw, size=4) {
    interim_adc_i1_i0: u32,
  }
  register InterimPidTorqueTargetFluxTargetTorqueActualFluxActualDiv256(addr = 0x6e, mode = rw, size=4) {
    interim_pid_torque_target_flux_target_torque_actual_flux_actual_div256: u32,
  }
  register InterimPidTorqueTargetTorqueActual(addr = 0x6e, mode = rw, size=4) {
    interim_pid_torque_target_torque_actual: u32,
  }
  register InterimPidFluxTargetFluxActual(addr = 0x6e, mode = rw, size=4) {
    interim_pid_flux_target_flux_actual: u32,
  }
  register InterimPidVelocityTargetVelocityActualDiv256(addr = 0x6e, mode = rw, size=4) {
    interim_pid_velocity_target_velocity_actual_div256: u32,
  }
  register InterimPidVelocityTargetVelocityActual(addr = 0x6e, mode = rw, size=4) {
    interim_pid_velocity_target_velocity_actual: u32,
  }
  register InterimPidPositionTargetPositionActualDiv256(addr = 0x6e, mode = rw, size=4) {
    interim_pid_position_target_position_actual_div256: u32,
  }
  register InterimPidPositionTargetPositionActual(addr = 0x6e, mode = rw, size=4) {
    interim_pid_position_target_position_actual: u32,
  }
  register InterimFfVelocity(addr = 0x6e, mode = rw, size=4) {
    interim_ff_velocity: u32,
  }
  register InterimFfTorque(addr = 0x6e, mode = rw, size=4) {
    interim_ff_torque: u32,
  }
  register InterimActualVelocityPptm(addr = 0x6e, mode = rw, size=4) {
    interim_actual_velocity_pptm: u32,
  }
  register InterimRefSwitchStatus(addr = 0x6e, mode = rw, size=4) {
    interim_ref_switch_status: u32,
  }
  register InterimHomePosition(addr = 0x6e, mode = rw, size=4) {
    interim_home_position: u32,
  }
  register InterimLeftPosition(addr = 0x6e, mode = rw, size=4) {
    interim_left_position: u32,
  }
  register InterimRightPosition(addr = 0x6e, mode = rw, size=4) {
    interim_right_position: u32,
  }
  register InterimSinglePinIfPwmDutyCycleTorqueTarget(addr = 0x6e, mode = rw, size=4) {
    interim_single_pin_if_pwm_duty_cycle_torque_target: u32,
  }
  register InterimSinglePinIfVelocityTarget(addr = 0x6e, mode = rw, size=4) {
    interim_single_pin_if_velocity_target: u32,
  }
  register InterimSinglePinIfPositionTarget(addr = 0x6e, mode = rw, size=4) {
    interim_single_pin_if_position_target: u32,
  }

  register AdcVmLimits(addr = 0x75, mode = rw, size=4) {
    adc_vm_limit_low: u16[0..16],
    adc_vm_limit_high: u16[16..32],
  }

  register Tmc4671InputsRaw(addr = 0x76, mode = rw, size=4) {
    tmc4671_inputs_raw: u32,
  }
  register Tmc4671OutputsRaw(addr = 0x77, mode = rw, size=4) {
    tmc4671_outputs_raw: u32,
  }
  register StepWidth(addr = 0x78, mode = rw, size=4) {
    step_width: u32,
  }
  register UartBps(addr = 0x79, mode = rw, size=4) {
    uart_bps: u32,
  }
  register GpioDsadciConfig(addr = 0x7b, mode = rw, size=4) {
    gpio_dsadci_config: u32,
  }

  register StatusFlags(addr = 0x7c, mode = rw, size=4) {
    pid_x_target_limit: bool[0],
    pid_x_errsum_limit: bool[2],
    pid_x_output_limit: bool[3],
    pid_v_target_limit: bool[4],
    pid_v_errsum_limit: bool[6],
    pid_v_output_limit: bool[7],
    pid_id_target_limit: bool[8],
    pid_id_errsum_limit: bool[10],
    pid_id_output_limit: bool[11],
    pid_iq_target_limit: bool[12],
    pid_iq_errsum_limit: bool[14],
    pid_iq_output_limit: bool[15],
    ipark_cirlim_limit_ud: bool[16],
    ipark_cirlim_limit_uq: bool[17],
    ipark_cirlim_limit_ur: bool[18],
    ref_sw_r: bool[20],
    ref_sw_h: bool[21],
    ref_sw_l: bool[22],
    pwm_min: bool[24],
    pwm_max: bool[25],
    adc_i_clipped: bool[26],
    aenc_clipped: bool[27],
    enc_n: bool[28],
    enc2_n: bool[29],
    aenc_n: bool[30],
    _: u8[1,5,9,13,19,23,31],
  }

  register StatusMask(addr = 0x7d, mode = rw, size=4) {
    pid_x_target_limit: bool[0],
    pid_x_errsum_limit: bool[2],
    pid_x_output_limit: bool[3],
    pid_v_target_limit: bool[4],
    pid_v_errsum_limit: bool[6],
    pid_v_output_limit: bool[7],
    pid_id_target_limit: bool[8],
    pid_id_errsum_limit: bool[10],
    pid_id_output_limit: bool[11],
    pid_iq_target_limit: bool[12],
    pid_iq_errsum_limit: bool[14],
    pid_iq_output_limit: bool[15],
    ipark_cirlim_limit_ud: bool[16],
    ipark_cirlim_limit_uq: bool[17],
    ipark_cirlim_limit_ur: bool[18],
    ref_sw_r: bool[20],
    ref_sw_h: bool[21],
    ref_sw_l: bool[22],
    pwm_min: bool[24],
    pwm_max: bool[25],
    adc_i_clipped: bool[26],
    aenc_clipped: bool[27],
    enc_n: bool[28],
    enc2_n: bool[29],
    aenc_n: bool[30],
    _: u8[1,5,9,13,19,23,31],
  }

}
