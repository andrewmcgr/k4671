use embedded_interfaces::codegen::interface_objects;

// pub type TMC4671I2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
// pub type TMC4671SpiCodec =
//     embedded_interfaces::registers::spi::codecs::standard_codec::StandardCodec<1, 6, 0, 7, false, 0>;

interface_objects! {
  register CHIPINFO_DATA(addr = 0xff, mode = rw, size=4) {
    chipinfo_data: u32,
  }

  register CHIPINFO_ADDR(addr = 0x1ff, mode = rw, size=4) {
    chipinfo_addr: u32,
  }

  register CHIPINFO_SI_TYPE(addr = 0x0, mode = rw, size=4) {
    chipinfo_si_type: u32,
  }

  register CHIPINFO_SI_VERSION(addr = 0x1, mode = rw, size=4) {
    chipinfo_si_version: u32,
  }

  register CHIPINFO_SI_DATE(addr = 0x2, mode = rw, size=4) {
    chipinfo_si_date: u32,
  }

  register CHIPINFO_SI_TIME(addr = 0x3, mode = rw, size=4) {
    chipinfo_si_time: u32,
  }

  register CHIPINFO_SI_VARIANT(addr = 0x4, mode = rw, size=4) {
    chipinfo_si_variant: u32,
  }

  register CHIPINFO_SI_BUILD(addr = 0x5, mode = rw, size=4) {
    chipinfo_si_build: u32,
  }

  register ADC_RAW_DATA(addr = 0x2ff, mode = rw, size=4) {
    adc_raw_data: u32,
  }

  register ADC_RAW_ADDR(addr = 0x3ff, mode = rw, size=4) {
    adc_raw_addr: u32,
  }

  register ADC_I1_RAW_ADC_I0_RAW(addr = 0x200, mode = rw, size=4) {
    adc_i0_raw: u16[0..16],
    adc_i1_raw: u16[16..32],
  }

  register ADC_AGPI_A_RAW_ADC_VM_RAW(addr = 0x201, mode = rw, size=4) {
    adc_agpi_a_raw: u16[0..16],
    adc_vm_raw: u16[16..32],
  }

  register ADC_AENC_UX_RAW_ADC_AGPI_B_RAW(addr = 0x202, mode = rw, size=4) {
    adc_aenc_ux_raw: u16[0..16],
    adc_agpi_b_raw: u16[16..32],
  }

  register ADC_AENC_WY_RAW_ADC_AENC_VN_RAW(addr = 0x203, mode = rw, size=4) {
    adc_aenc_wy_raw: u16[0..16],
    adc_aenc_vn_raw: u16[16..32],
  }

  register DSADC_MCFG_B_MCFG_A(addr = 0x4ff, mode = rw, size=4) {
    cfg_dsmodulator_a: u2[0..2],
    mclk_polarity_a: bool[2],
    mdat_polarity_a: bool[3],
    sel_nclk_mclk_i_a: bool[4],
    cfg_dsmodulator_b: u2[16..18],
    mclk_polarity_b: bool[18],
    mdat_polarity_b: bool[19],
    sel_nclk_mclk_i_b: bool[20],
  }

  register DSADC_MCLK_A(addr = 0x5ff, mode = rw, size=4) {
    dsadc_mclk_a: u32,
  }

  register DSADC_MCLK_B(addr = 0x6ff, mode = rw, size=4) {
    dsadc_mclk_b: u32,
  }

  register DSADC_MDEC_B_MDEC_A(addr = 0x7ff, mode = rw, size=4) {
    dsadc_mdec_a: u16[0..16],
    dsadc_mdec_b: u16[16..32],
  }

  register ADC_I1_SCALE_OFFSET(addr = 0x8ff, mode = rw, size=4) {
    adc_i1_offset: u16[0..16],
    adc_i1_scale: u16[16..32],
  }

  register ADC_I0_SCALE_OFFSET(addr = 0x9ff, mode = rw, size=4) {
    adc_i0_offset: u16[0..16],
    adc_i0_scale: u16[16..32],
  }

  register ADC_I_SELECT(addr = 0xaff, mode = rw, size=4) {
    adc_i0_select: u8[0..8],
    adc_i1_select: u8[8..16],
    adc_i_ux_select: u2[24..26],
    adc_i_v_select: u2[26..28],
    adc_i_wy_select: u2[28..30],
  }

  register ADC_I1_I0_EXT(addr = 0xbff, mode = rw, size=4) {
    adc_i0_ext: u16[0..16],
    adc_i1_ext: u16[16..32],
  }

  register DS_ANALOG_INPUT_STAGE_CFG(addr = 0xcff, mode = rw, size=4) {
    cfg_adc_i0: u4[0..4],
    cfg_adc_i1: u4[4..8],
    cfg_adc_vm: u4[8..12],
    cfg_adc_agpi_a: u4[12..16],
    cfg_adc_agpi_b: u4[16..20],
    cfg_adc_aenc_ux: u4[20..24],
    cfg_adc_aenc_vn: u4[24..28],
    cfg_adc_aenc_wy: u4[28..32],
  }

  register AENC_0_SCALE_OFFSET(addr = 0xdff, mode = rw, size=4) {
    aenc_0_offset: u16[0..16],
    aenc_0_scale: u16[16..32],
  }

  register AENC_1_SCALE_OFFSET(addr = 0xeff, mode = rw, size=4) {
    aenc_1_offset: u16[0..16],
    aenc_1_scale: u16[16..32],
  }

  register AENC_2_SCALE_OFFSET(addr = 0xfff, mode = rw, size=4) {
    aenc_2_offset: u16[0..16],
    aenc_2_scale: u16[16..32],
  }

  register AENC_SELECT(addr = 0x11ff, mode = rw, size=4) {
    aenc_0_select: u8[0..8],
    aenc_1_select: u8[8..16],
    aenc_2_select: u8[16..24],
  }

  register ADC_IWY_IUX(addr = 0x12ff, mode = rw, size=4) {
    adc_iux: u16[0..16],
    adc_iwy: u16[16..32],
  }

  register ADC_IV(addr = 0x13ff, mode = rw, size=4) {
    adc_iv: u16[0..16],
  }

  register AENC_WY_UX(addr = 0x15ff, mode = rw, size=4) {
    aenc_ux: u16[0..16],
    aenc_wy: u16[16..32],
  }

  register AENC_VN(addr = 0x16ff, mode = rw, size=4) {
    aenc_vn: u16[0..16],
  }

  register PWM_POLARITIES(addr = 0x17ff, mode = rw, size=4) {
    pwm_polarities_0: bool[0],
    pwm_polarities_1: bool[1],
  }

  register PWM_MAXCNT(addr = 0x18ff, mode = rw, size=4) {
    pwm_maxcnt: u32,
  }

  register PWM_BBM_H_BBM_L(addr = 0x19ff, mode = rw, size=4) {
    pwm_bbm_l: u8[0..8],
    pwm_bbm_h: u8[8..16],
  }

  register PWM_SV_CHOP(addr = 0x1aff, mode = rw, size=4) {
    pwm_chop: u8[0..8],
    pwm_sv: bool[8],
  }

  register MOTOR_TYPE_N_POLE_PAIRS(addr = 0x1bff, mode = rw, size=4) {
    n_pole_pairs: u16[0..16],
    motor_type: u8[16..24],
  }

  register PHI_E_EXT(addr = 0x1cff, mode = rw, size=4) {
    phi_e_ext: u32,
  }

  register OPENLOOP_MODE(addr = 0x1fff, mode = rw, size=4) {
    openloop_phi_direction: bool[12],
  }

  register OPENLOOP_ACCELERATION(addr = 0x20ff, mode = rw, size=4) {
    openloop_acceleration: u32,
  }

  register OPENLOOP_VELOCITY_TARGET(addr = 0x21ff, mode = rw, size=4) {
    openloop_velocity_target: u32,
  }

  register OPENLOOP_VELOCITY_ACTUAL(addr = 0x22ff, mode = rw, size=4) {
    openloop_velocity_actual: u32,
  }

  register OPENLOOP_PHI(addr = 0x23ff, mode = rw, size=4) {
    openloop_phi: u32,
  }

  register UQ_UD_EXT(addr = 0x24ff, mode = rw, size=4) {
    ud_ext: u16[0..16],
    uq_ext: u16[16..32],
  }

  register ABN_DECODER_MODE(addr = 0x25ff, mode = rw, size=4) {
    abn_apol: bool[0],
    abn_bpol: bool[1],
    abn_npol: bool[2],
    abn_use_abn_as_n: bool[3],
    abn_cln: bool[8],
    abn_direction: bool[12],
  }

  register ABN_DECODER_PPR(addr = 0x26ff, mode = rw, size=4) {
    abn_decoder_ppr: u32,
  }

  register ABN_DECODER_COUNT(addr = 0x27ff, mode = rw, size=4) {
    abn_decoder_count: u32,
  }

  register ABN_DECODER_COUNT_N(addr = 0x28ff, mode = rw, size=4) {
    abn_decoder_count_n: u32,
  }

  register ABN_DECODER_PHI_E_PHI_M_OFFSET(addr = 0x29ff, mode = rw, size=4) {
    abn_decoder_phi_m_offset: u16[0..16],
    abn_decoder_phi_e_offset: u16[16..32],
  }

  register ABN_DECODER_PHI_E_PHI_M(addr = 0x2aff, mode = rw, size=4) {
    abn_decoder_phi_m: u16[0..16],
    abn_decoder_phi_e: u16[16..32],
  }

  register ABN_2_DECODER_MODE(addr = 0x2cff, mode = rw, size=4) {
    abn_2_apol: bool[0],
    abn_2_bpol: bool[1],
    abn_2_npol: bool[2],
    abn_2_use_abn_as_n: bool[3],
    abn_2_cln: bool[8],
    abn_2_direction: bool[12],
  }

  register ABN_2_DECODER_PPR(addr = 0x2dff, mode = rw, size=4) {
    abn_2_decoder_ppr: u32,
  }

  register ABN_2_DECODER_COUNT(addr = 0x2eff, mode = rw, size=4) {
    abn_2_decoder_count: u32,
  }

  register ABN_2_DECODER_COUNT_N(addr = 0x2fff, mode = rw, size=4) {
    abn_2_decoder_count_n: u32,
  }

  register ABN_2_DECODER_PHI_M_OFFSET(addr = 0x30ff, mode = rw, size=4) {
    abn_2_decoder_phi_m_offset: u32,
  }

  register ABN_2_DECODER_PHI_M(addr = 0x31ff, mode = rw, size=4) {
    abn_2_decoder_phi_m: u32,
  }

  register HALL_MODE(addr = 0x33ff, mode = rw, size=4) {
    hall_polarity: bool[0],
    hall_sync: bool[4],
    hall_interp: bool[8],
    hall_dir: bool[12],
    hall_blank: u16[16..28],
  }

  register HALL_POSITION_060_000(addr = 0x34ff, mode = rw, size=4) {
    hall_position_000: u16[0..16],
    hall_position_060: u16[16..32],
  }

  register HALL_POSITION_180_120(addr = 0x35ff, mode = rw, size=4) {
    hall_position_120: u16[0..16],
    hall_position_180: u16[16..32],
  }

  register HALL_POSITION_300_240(addr = 0x36ff, mode = rw, size=4) {
    hall_position_240: u16[0..16],
    hall_position_300: u16[16..32],
  }

  register HALL_PHI_E_PHI_M_OFFSET(addr = 0x37ff, mode = rw, size=4) {
    hall_phi_m_offset: u16[0..16],
    hall_phi_e_offset: u16[16..32],
  }

  register HALL_DPHI_MAX(addr = 0x38ff, mode = rw, size=4) {
    hall_dphi_max: u32,
  }

  register HALL_PHI_E_INTERPOLATED_PHI_E(addr = 0x39ff, mode = rw, size=4) {
    hall_phi_e: u16[0..16],
    hall_phi_e_interpolated: u16[16..32],
  }

  register HALL_PHI_M(addr = 0x3aff, mode = rw, size=4) {
    hall_phi_m: u32,
  }

  register AENC_DECODER_MODE(addr = 0x3bff, mode = rw, size=4) {
    aenc_deg: bool[0],
    aenc_dir: bool[12],
  }

  register AENC_DECODER_N_THRESHOLD(addr = 0x3cff, mode = rw, size=4) {
    aenc_decoder_n_threshold: u32,
  }

  register AENC_DECODER_PHI_A_RAW(addr = 0x3dff, mode = rw, size=4) {
    aenc_decoder_phi_a_raw: u32,
  }

  register AENC_DECODER_PHI_A_OFFSET(addr = 0x3eff, mode = rw, size=4) {
    aenc_decoder_phi_a_offset: u32,
  }

  register AENC_DECODER_PHI_A(addr = 0x3fff, mode = rw, size=4) {
    aenc_decoder_phi_a: u32,
  }

  register AENC_DECODER_PPR(addr = 0x40ff, mode = rw, size=4) {
    aenc_ppr: u16[0..16],
  }

  register AENC_DECODER_COUNT(addr = 0x41ff, mode = rw, size=4) {
    aenc_decoder_count: u32,
  }

  register AENC_DECODER_COUNT_N(addr = 0x42ff, mode = rw, size=4) {
    aenc_decoder_count_n: u32,
  }

  register AENC_DECODER_PHI_E_PHI_M_OFFSET(addr = 0x45ff, mode = rw, size=4) {
    aenc_decoder_phi_m_offset: u16[0..16],
    aenc_decoder_phi_e_offset: u16[16..32],
  }

  register AENC_DECODER_PHI_E_PHI_M(addr = 0x46ff, mode = rw, size=4) {
    aenc_decoder_phi_m: u16[0..16],
    aenc_decoder_phi_e: u16[16..32],
  }

  register CONFIG_DATA(addr = 0x4dff, mode = rw, size=4) {
    config_data: u32,
  }

  register CONFIG_ADDR(addr = 0x4eff, mode = rw, size=4) {
    config_addr: u32,
  }

  register CONFIG_BIQUAD_X_A_1(addr = 0x4d01, mode = rw, size=4) {
    config_biquad_x_a_1: u32,
  }

  register CONFIG_BIQUAD_X_A_2(addr = 0x4d02, mode = rw, size=4) {
    config_biquad_x_a_2: u32,
  }

  register CONFIG_BIQUAD_X_B_0(addr = 0x4d04, mode = rw, size=4) {
    config_biquad_x_b_0: u32,
  }

  register CONFIG_BIQUAD_X_B_1(addr = 0x4d05, mode = rw, size=4) {
    config_biquad_x_b_1: u32,
  }

  register CONFIG_BIQUAD_X_B_2(addr = 0x4d06, mode = rw, size=4) {
    config_biquad_x_b_2: u32,
  }

  register CONFIG_BIQUAD_X_ENABLE(addr = 0x4d07, mode = rw, size=4) {
    config_biquad_x_enable: u32,
  }

  register CONFIG_BIQUAD_V_A_1(addr = 0x4d09, mode = rw, size=4) {
    config_biquad_v_a_1: u32,
  }

  register CONFIG_BIQUAD_V_A_2(addr = 0x4d0a, mode = rw, size=4) {
    config_biquad_v_a_2: u32,
  }

  register CONFIG_BIQUAD_V_B_0(addr = 0x4d0c, mode = rw, size=4) {
    config_biquad_v_b_0: u32,
  }

  register CONFIG_BIQUAD_V_B_1(addr = 0x4d0d, mode = rw, size=4) {
    config_biquad_v_b_1: u32,
  }

  register CONFIG_BIQUAD_V_B_2(addr = 0x4d0e, mode = rw, size=4) {
    config_biquad_v_b_2: u32,
  }

  register CONFIG_BIQUAD_V_ENABLE(addr = 0x4d0f, mode = rw, size=4) {
    config_biquad_v_enable: u32,
  }

  register CONFIG_BIQUAD_T_A_1(addr = 0x4d11, mode = rw, size=4) {
    config_biquad_t_a_1: u32,
  }

  register CONFIG_BIQUAD_T_A_2(addr = 0x4d12, mode = rw, size=4) {
    config_biquad_t_a_2: u32,
  }

  register CONFIG_BIQUAD_T_B_0(addr = 0x4d14, mode = rw, size=4) {
    config_biquad_t_b_0: u32,
  }

  register CONFIG_BIQUAD_T_B_1(addr = 0x4d15, mode = rw, size=4) {
    config_biquad_t_b_1: u32,
  }

  register CONFIG_BIQUAD_T_B_2(addr = 0x4d16, mode = rw, size=4) {
    config_biquad_t_b_2: u32,
  }

  register CONFIG_BIQUAD_T_ENABLE(addr = 0x4d17, mode = rw, size=4) {
    config_biquad_t_enable: u32,
  }

  register CONFIG_BIQUAD_F_A_1(addr = 0x4d19, mode = rw, size=4) {
    config_biquad_f_a_1: u32,
  }

  register CONFIG_BIQUAD_F_A_2(addr = 0x4d1a, mode = rw, size=4) {
    config_biquad_f_a_2: u32,
  }

  register CONFIG_BIQUAD_F_B_0(addr = 0x4d1c, mode = rw, size=4) {
    config_biquad_f_b_0: u32,
  }

  register CONFIG_BIQUAD_F_B_1(addr = 0x4d1d, mode = rw, size=4) {
    config_biquad_f_b_1: u32,
  }

  register CONFIG_BIQUAD_F_B_2(addr = 0x4d1e, mode = rw, size=4) {
    config_biquad_f_b_2: u32,
  }

  register CONFIG_BIQUAD_F_ENABLE(addr = 0x4d1f, mode = rw, size=4) {
    config_biquad_f_enable: u32,
  }

  register FEED_FORWARD_VELOCITY_GAIN(addr = 0x4d28, mode = rw, size=4) {
    feed_forward_velocity_gain: u32,
  }

  register FEED_FORWARD_VELOCITY_FILTER_CONSTANT(addr = 0x4d29, mode = rw, size=4) {
    feed_forward_velocity_filter_constant: u32,
  }

  register FEED_FORWARD_TORQUE_GAIN(addr = 0x4d2a, mode = rw, size=4) {
    feed_forward_torque_gain: u32,
  }

  register FEED_FORWARD_TORQUE_FILTER_CONSTANT(addr = 0x4d2b, mode = rw, size=4) {
    feed_forward_torque_filter_constant: u32,
  }

  register CONFIG_REF_SWITCH_CONFIG(addr = 0x4d33, mode = rw, size=4) {
    config_ref_switch_config: u32,
  }

  register CONFIG_SINGLE_PIN_IF_STATUS_CFG(addr = 0x4d3c, mode = rw, size=4) {
    config_single_pin_if_status_cfg: u32,
  }

  register CONFIG_SINGLE_PIN_IF_SCALE_OFFSET(addr = 0x4d3d, mode = rw, size=4) {
    config_single_pin_if_scale_offset: u32,
  }

  register CONFIG_ADVANCED_PI_REPRESENT(addr = 0x4d3e, mode = rw, size=4) {
    current_i_n: bool[0],
    current_p_n: bool[1],
    velocity_i_n: bool[2],
    velocity_p_n: bool[3],
    position_i_n: bool[4],
    position_p_n: bool[5],
  }

  register VELOCITY_SELECTION(addr = 0x50ff, mode = rw, size=4) {
    velocity_selection: u8[0..8],
    velocity_meter_selection: u8[8..16],
  }

  register POSITION_SELECTION(addr = 0x51ff, mode = rw, size=4) {
    position_selection: u32,
  }

  register PHI_E_SELECTION(addr = 0x52ff, mode = rw, size=4) {
    phi_e_selection: u32,
  }

  register PHI_E(addr = 0x53ff, mode = rw, size=4) {
    phi_e: u32,
  }

  register PID_FLUX_P_FLUX_I(addr = 0x54ff, mode = rw, size=4) {
    pid_flux_i: u16[0..16],
    pid_flux_p: u16[16..32],
  }

  register PID_TORQUE_P_TORQUE_I(addr = 0x56ff, mode = rw, size=4) {
    pid_torque_i: u16[0..16],
    pid_torque_p: u16[16..32],
  }

  register PID_VELOCITY_P_VELOCITY_I(addr = 0x58ff, mode = rw, size=4) {
    pid_velocity_i: u16[0..16],
    pid_velocity_p: u16[16..32],
  }

  register PID_POSITION_P_POSITION_I(addr = 0x5aff, mode = rw, size=4) {
    pid_position_i: u16[0..16],
    pid_position_p: u16[16..32],
  }

  register PIDOUT_UQ_UD_LIMITS(addr = 0x5dff, mode = rw, size=4) {
    pidout_uq_ud_limits: u32,
  }

  register PID_TORQUE_FLUX_LIMITS(addr = 0x5eff, mode = rw, size=4) {
    pid_torque_flux_limits: u32,
  }

  register PID_VELOCITY_LIMIT(addr = 0x60ff, mode = rw, size=4) {
    pid_velocity_limit: u32,
  }

  register PID_POSITION_LIMIT_LOW(addr = 0x61ff, mode = rw, size=4) {
    pid_position_limit_low: u32,
  }

  register PID_POSITION_LIMIT_HIGH(addr = 0x62ff, mode = rw, size=4) {
    pid_position_limit_high: u32,
  }

  register MODE_RAMP_MODE_MOTION(addr = 0x63ff, mode = rw, size=4) {
    mode_motion: u8[0..8],
    mode_pid_smpl: u8[24..31],
    mode_ff: u8[16..24],
    mode_pid_type: bool[31],
  }

  register PID_TORQUE_FLUX_TARGET(addr = 0x64ff, mode = rw, size=4) {
    pid_flux_target: u16[0..16],
    pid_torque_target: u16[16..32],
  }

  register PID_TORQUE_FLUX_OFFSET(addr = 0x65ff, mode = rw, size=4) {
    pid_flux_offset: u16[0..16],
    pid_torque_offset: u16[16..32],
  }

  register PID_VELOCITY_TARGET(addr = 0x66ff, mode = rw, size=4) {
    pid_velocity_target: u32,
  }

  register PID_VELOCITY_OFFSET(addr = 0x67ff, mode = rw, size=4) {
    pid_velocity_offset: u32,
  }

  register PID_POSITION_TARGET(addr = 0x68ff, mode = rw, size=4) {
    pid_position_target: u32,
  }

  register PID_TORQUE_FLUX_ACTUAL(addr = 0x69ff, mode = rw, size=4) {
    pid_flux_actual: u16[0..16],
    pid_torque_actual: u16[16..32],
  }

  register PID_VELOCITY_ACTUAL(addr = 0x6aff, mode = rw, size=4) {
    pid_velocity_actual: u32,
  }

  register PID_POSITION_ACTUAL(addr = 0x6bff, mode = rw, size=4) {
    pid_position_actual: u32,
  }

  register PID_ERROR_DATA(addr = 0x6cff, mode = rw, size=4) {
    pid_error_data: u32,
  }

  register PID_ERROR_ADDR(addr = 0x6dff, mode = rw, size=4) {
    pid_error_addr: u32,
  }

  register PID_ERROR_PID_TORQUE_ERROR(addr = 0x6c00, mode = rw, size=4) {
    pid_error_pid_torque_error: u32,
  }

  register PID_ERROR_PID_FLUX_ERROR(addr = 0x6c01, mode = rw, size=4) {
    pid_error_pid_flux_error: u32,
  }

  register PID_ERROR_PID_VELOCITY_ERROR(addr = 0x6c02, mode = rw, size=4) {
    pid_error_pid_velocity_error: u32,
  }

  register PID_ERROR_PID_POSITION_ERROR(addr = 0x6c03, mode = rw, size=4) {
    pid_error_pid_position_error: u32,
  }

  register PID_ERROR_PID_TORQUE_ERROR_SUM(addr = 0x6c04, mode = rw, size=4) {
    pid_error_pid_torque_error_sum: u32,
  }

  register PID_ERROR_PID_FLUX_ERROR_SUM(addr = 0x6c05, mode = rw, size=4) {
    pid_error_pid_flux_error_sum: u32,
  }

  register PID_ERROR_PID_VELOCITY_ERROR_SUM(addr = 0x6c06, mode = rw, size=4) {
    pid_error_pid_velocity_error_sum: u32,
  }

  register PID_ERROR_PID_POSITION_ERROR_SUM(addr = 0x6c07, mode = rw, size=4) {
    pid_error_pid_position_error_sum: u32,
  }

  register INTERIM_DATA(addr = 0x6eff, mode = rw, size=4) {
    interim_data: u32,
  }

  register INTERIM_ADDR(addr = 0x6fff, mode = rw, size=4) {
    interim_addr: u32,
  }

  register INTERIM_PIDIN_TARGET_TORQUE(addr = 0x6e00, mode = rw, size=4) {
    interim_pidin_target_torque: u32,
  }

  register INTERIM_PIDIN_TARGET_FLUX(addr = 0x6e01, mode = rw, size=4) {
    interim_pidin_target_flux: u32,
  }

  register INTERIM_PIDIN_TARGET_VELOCITY(addr = 0x6e02, mode = rw, size=4) {
    interim_pidin_target_velocity: u32,
  }

  register INTERIM_PIDIN_TARGET_POSITION(addr = 0x6e03, mode = rw, size=4) {
    interim_pidin_target_position: u32,
  }

  register INTERIM_PIDOUT_TARGET_TORQUE(addr = 0x6e04, mode = rw, size=4) {
    interim_pidout_target_torque: u32,
  }

  register INTERIM_PIDOUT_TARGET_FLUX(addr = 0x6e05, mode = rw, size=4) {
    interim_pidout_target_flux: u32,
  }

  register INTERIM_PIDOUT_TARGET_VELOCITY(addr = 0x6e06, mode = rw, size=4) {
    interim_pidout_target_velocity: u32,
  }

  register INTERIM_PIDOUT_TARGET_POSITION(addr = 0x6e07, mode = rw, size=4) {
    interim_pidout_target_position: u32,
  }

  register INTERIM_FOC_IWY_IUX(addr = 0x6e08, mode = rw, size=4) {
    interim_foc_iwy_iux: u32,
  }

  register INTERIM_FOC_IV(addr = 0x6e09, mode = rw, size=4) {
    interim_foc_iv: u32,
  }

  register INTERIM_FOC_IB_IA(addr = 0x6e0a, mode = rw, size=4) {
    interim_foc_ib_ia: u32,
  }

  register INTERIM_FOC_IQ_ID(addr = 0x6e0b, mode = rw, size=4) {
    interim_foc_iq_id: u32,
  }

  register INTERIM_FOC_UQ_UD(addr = 0x6e0c, mode = rw, size=4) {
    interim_foc_uq_ud: u32,
  }

  register INTERIM_FOC_UQ_UD_LIMITED(addr = 0x6e0d, mode = rw, size=4) {
    interim_foc_uq_ud_limited: u32,
  }

  register INTERIM_FOC_UB_UA(addr = 0x6e0e, mode = rw, size=4) {
    interim_foc_ub_ua: u32,
  }

  register INTERIM_FOC_UWY_UUX(addr = 0x6e0f, mode = rw, size=4) {
    interim_foc_uwy_uux: u32,
  }

  register INTERIM_FOC_UV(addr = 0x6e10, mode = rw, size=4) {
    interim_foc_uv: u32,
  }

  register INTERIM_PWM_WY_UX(addr = 0x6e11, mode = rw, size=4) {
    interim_pwm_ux: u16[0..16],
    interim_pwm_wy: u16[16..32],
  }

  register INTERIM_PWM_UV(addr = 0x6e12, mode = rw, size=4) {
    interim_pwm_uv: u32,
  }

  register INTERIM_ADC_I1_I0(addr = 0x6e13, mode = rw, size=4) {
    interim_adc_i1_i0: u32,
  }

  register INTERIM_PID_TORQUE_TARGET_FLUX_TARGET_TORQUE_ACTUAL_FLUX_ACTUAL_DIV256(addr = 0x6e14, mode = rw, size=4) {
    interim_pid_torque_target_flux_target_torque_actual_flux_actual_div256: u32,
  }

  register INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL(addr = 0x6e15, mode = rw, size=4) {
    interim_pid_torque_target_torque_actual: u32,
  }

  register INTERIM_PID_FLUX_TARGET_FLUX_ACTUAL(addr = 0x6e16, mode = rw, size=4) {
    interim_pid_flux_target_flux_actual: u32,
  }

  register INTERIM_PID_VELOCITY_TARGET_VELOCITY_ACTUAL_DIV256(addr = 0x6e17, mode = rw, size=4) {
    interim_pid_velocity_target_velocity_actual_div256: u32,
  }

  register INTERIM_PID_VELOCITY_TARGET_VELOCITY_ACTUAL(addr = 0x6e18, mode = rw, size=4) {
    interim_pid_velocity_target_velocity_actual: u32,
  }

  register INTERIM_PID_POSITION_TARGET_POSITION_ACTUAL_DIV256(addr = 0x6e19, mode = rw, size=4) {
    interim_pid_position_target_position_actual_div256: u32,
  }

  register INTERIM_PID_POSITION_TARGET_POSITION_ACTUAL(addr = 0x6e1a, mode = rw, size=4) {
    interim_pid_position_target_position_actual: u32,
  }

  register INTERIM_FF_VELOCITY(addr = 0x6e1b, mode = rw, size=4) {
    interim_ff_velocity: u32,
  }

  register INTERIM_FF_TORQUE(addr = 0x6e1c, mode = rw, size=4) {
    interim_ff_torque: u32,
  }

  register INTERIM_ACTUAL_VELOCITY_PPTM(addr = 0x6e1d, mode = rw, size=4) {
    interim_actual_velocity_pptm: u32,
  }

  register INTERIM_REF_SWITCH_STATUS(addr = 0x6e1e, mode = rw, size=4) {
    interim_ref_switch_status: u32,
  }

  register INTERIM_HOME_POSITION(addr = 0x6e1f, mode = rw, size=4) {
    interim_home_position: u32,
  }

  register INTERIM_LEFT_POSITION(addr = 0x6e20, mode = rw, size=4) {
    interim_left_position: u32,
  }

  register INTERIM_RIGHT_POSITION(addr = 0x6e21, mode = rw, size=4) {
    interim_right_position: u32,
  }

  register INTERIM_SINGLE_PIN_IF_PWM_DUTY_CYCLE_TORQUE_TARGET(addr = 0x6e2a, mode = rw, size=4) {
    interim_single_pin_if_pwm_duty_cycle_torque_target: u32,
  }

  register INTERIM_SINGLE_PIN_IF_VELOCITY_TARGET(addr = 0x6e2b, mode = rw, size=4) {
    interim_single_pin_if_velocity_target: u32,
  }

  register INTERIM_SINGLE_PIN_IF_POSITION_TARGET(addr = 0x6e2c, mode = rw, size=4) {
    interim_single_pin_if_position_target: u32,
  }

  register ADC_VM_LIMITS(addr = 0x75ff, mode = rw, size=4) {
    adc_vm_limit_low: u16[0..16],
    adc_vm_limit_high: u16[16..32],
  }

  register TMC4671_INPUTS_RAW(addr = 0x76ff, mode = rw, size=4) {
    tmc4671_inputs_raw: u32,
  }

  register TMC4671_OUTPUTS_RAW(addr = 0x77ff, mode = rw, size=4) {
    tmc4671_outputs_raw: u32,
  }

  register STEP_WIDTH(addr = 0x78ff, mode = rw, size=4) {
    step_width: u32,
  }

  register UART_BPS(addr = 0x79ff, mode = rw, size=4) {
    uart_bps: u32,
  }

  register GPIO_DSADCI_CONFIG(addr = 0x7bff, mode = rw, size=4) {
    gpio_dsadci_config: u32,
  }

  register STATUS_FLAGS(addr = 0x7cff, mode = rw, size=4) {
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
    ipark_cirlim_limit_u_d: bool[16],
    ipark_cirlim_limit_u_q: bool[17],
    ipark_cirlim_limit_u_r: bool[18],
    ref_sw_r: bool[20],
    ref_sw_h: bool[21],
    ref_sw_l: bool[22],
    pwm_min: bool[24],
    pwm_max: bool[25],
    adc_i_clipped: bool[26],
    aenc_clipped: bool[27],
    enc_n: bool[28],
    enc_2_n: bool[29],
    aenc_n: bool[30],
  }

  register STATUS_MASK(addr = 0x7dff, mode = rw, size=4) {
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
    ipark_cirlim_limit_u_d: bool[16],
    ipark_cirlim_limit_u_q: bool[17],
    ipark_cirlim_limit_u_r: bool[18],
    ref_sw_r: bool[20],
    ref_sw_h: bool[21],
    ref_sw_l: bool[22],
    pwm_min: bool[24],
    pwm_max: bool[25],
    adc_i_clipped: bool[26],
    aenc_clipped: bool[27],
    enc_n: bool[28],
    enc_2_n: bool[29],
    aenc_n: bool[30],
  }

}
