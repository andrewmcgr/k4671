use embedded_interfaces::codegen::interface_objects;


pub type TMC4671I2cCodec = embedded_interfaces::registers::i2c::codecs::unsupported_codec::UnsupportedCodec<()>;
pub type TMC4671SpiCodec =
    embedded_interfaces::registers::spi::codecs::standard_codec::StandardCodec<1, 6, 0, 7, false, 0>;


interface_objects! {
  register ChipinfoData(addr = 0x0, mode = rw, size=4) {
    ChipinfoData: u32,
  }

  enum ChipinfoAddr: u32{32} {
    0 ChipinfoSiType,
    1 ChipinfoSiVersion,
    2 ChipinfoSiDate,
    3 ChipinfoSiTime,
    4 ChipinfoSiVariant,
    5 ChipinfoSiBuild,
  }

  register ChipinfoAddr(addr = 0x1, mode = rw, size=4) {
    ChipinfoAddr: u32,
  }

  register ChipinfoSiType(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiType: u32,
  }

  register ChipinfoSiVersion(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiVersion: u32,
  }

  register ChipinfoSiDate(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiDate: u32,
  }

  register ChipinfoSiTime(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiTime: u32,
  }

  register ChipinfoSiVariant(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiVariant: u32,
  }

  register ChipinfoSiBuild(addr = 0x0, mode = rw, size=4) {
    ChipinfoSiBuild: u32,
  }

  register AdcRawData(addr = 0x2, mode = rw, size=4) {
    AdcRawData: u32,
  }

  enum AdcRawAddr: u32{32} {
    0 AdcI1RawAdcI0Raw,
    1 AdcAgpiARawAdcVmRaw,
    2 AdcAencUxRawAdcAgpiBRaw,
    3 AdcAencWyRawAdcAencVnRaw,
  }

  register AdcRawAddr(addr = 0x3, mode = rw, size=4) {
    AdcRawAddr: u32,
  }

  register AdcI1RawAdcI0Raw(addr = 0x2, mode = rw, size=4) {
    AdcI0Raw: u16[0..16],
    AdcI1Raw: u16[16..32],
  }

  register AdcAgpiARawAdcVmRaw(addr = 0x2, mode = rw, size=4) {
    AdcAgpiARaw: u16[0..16],
    AdcVmRaw: u16[16..32],
  }

  register AdcAencUxRawAdcAgpiBRaw(addr = 0x2, mode = rw, size=4) {
    AdcAencUxRaw: u16[0..16],
    AdcAgpiBRaw: u16[16..32],
  }

  register AdcAencWyRawAdcAencVnRaw(addr = 0x2, mode = rw, size=4) {
    AdcAencWyRaw: u16[0..16],
    AdcAencVnRaw: u16[16..32],
  }

  register DsadcMcfgBMcfgA(addr = 0x4, mode = rw, size=4) {
    CfgDsmodulatorA: u8[0..2],
    MclkPolarityA: bool[2],
    MdatPolarityA: bool[3],
    SelNclkMclkIA: bool[4],
    CfgDsmodulatorB: u8[16..18],
    MclkPolarityB: bool[18],
    MdatPolarityB: bool[19],
    SelNclkMclkIB: bool[20],
  }

  register DsadcMclkA(addr = 0x5, mode = rw, size=4) {
    DsadcMclkA: u32,
  }

  register DsadcMclkB(addr = 0x6, mode = rw, size=4) {
    DsadcMclkB: u32,
  }

  register DsadcMdecBMdecA(addr = 0x7, mode = rw, size=4) {
    DsadcMdecA: u16[0..16],
    DsadcMdecB: u16[16..32],
  }

  register AdcI1ScaleOffset(addr = 0x8, mode = rw, size=4) {
    AdcI1Offset: u16[0..16],
    AdcI1Scale: u16[16..32],
  }

  register AdcI0ScaleOffset(addr = 0x9, mode = rw, size=4) {
    AdcI0Offset: u16[0..16],
    AdcI0Scale: u16[16..32],
  }

  register AdcISelect(addr = 0xa, mode = rw, size=4) {
    AdcI0Select: u8[0..8],
    AdcI1Select: u8[8..16],
    AdcIUxSelect: u8[24..26],
    AdcIVSelect: u8[26..28],
    AdcIWySelect: u8[28..30],
  }

  register AdcI1I0Ext(addr = 0xb, mode = rw, size=4) {
    AdcI0Ext: u16[0..16],
    AdcI1Ext: u16[16..32],
  }

  register DsAnalogInputStageCfg(addr = 0xc, mode = rw, size=4) {
    CfgAdcI0: u8[0..4],
    CfgAdcI1: u8[4..8],
    CfgAdcVm: u8[8..12],
    CfgAdcAgpiA: u8[12..16],
    CfgAdcAgpiB: u8[16..20],
    CfgAdcAencUx: u8[20..24],
    CfgAdcAencVn: u8[24..28],
    CfgAdcAencWy: u8[28..32],
  }

  register Aenc0ScaleOffset(addr = 0xd, mode = rw, size=4) {
    Aenc0Offset: u16[0..16],
    Aenc0Scale: u16[16..32],
  }

  register Aenc1ScaleOffset(addr = 0xe, mode = rw, size=4) {
    Aenc1Offset: u16[0..16],
    Aenc1Scale: u16[16..32],
  }

  register Aenc2ScaleOffset(addr = 0xf, mode = rw, size=4) {
    Aenc2Offset: u16[0..16],
    Aenc2Scale: u16[16..32],
  }

  register AencSelect(addr = 0x11, mode = rw, size=4) {
    Aenc0Select: u8[0..8],
    Aenc1Select: u8[8..16],
    Aenc2Select: u8[16..24],
  }

  register AdcIwyIux(addr = 0x12, mode = rw, size=4) {
    AdcIux: u16[0..16],
    AdcIwy: u16[16..32],
  }

  register AdcIv(addr = 0x13, mode = rw, size=4) {
    AdcIv: u16[0..16],
  }

  register AencWyUx(addr = 0x15, mode = rw, size=4) {
    AencUx: u16[0..16],
    AencWy: u16[16..32],
  }

  register AencVn(addr = 0x16, mode = rw, size=4) {
    AencVn: u16[0..16],
  }

  register PwmPolarities(addr = 0x17, mode = rw, size=4) {
    PwmPolarities0: bool[0],
    PwmPolarities1: bool[1],
  }

  register PwmMaxcnt(addr = 0x18, mode = rw, size=4) {
    PwmMaxcnt: u32,
  }

  register PwmBbmHBbmL(addr = 0x19, mode = rw, size=4) {
    PwmBbmL: u8[0..8],
    PwmBbmH: u8[8..16],
  }

  register PwmSvChop(addr = 0x1a, mode = rw, size=4) {
    PwmChop: u8[0..8],
    PwmSv: bool[8],
  }

  register MotorTypeNPolePairs(addr = 0x1b, mode = rw, size=4) {
    NPolePairs: u16[0..16],
    MotorType: u8[16..24],
  }

  register PhiEExt(addr = 0x1c, mode = rw, size=4) {
    PhiEExt: u32,
  }

  register OpenloopMode(addr = 0x1f, mode = rw, size=4) {
    OpenloopPhiDirection: bool[12],
  }

  register OpenloopAcceleration(addr = 0x20, mode = rw, size=4) {
    OpenloopAcceleration: u32,
  }

  register OpenloopVelocityTarget(addr = 0x21, mode = rw, size=4) {
    OpenloopVelocityTarget: u32,
  }

  register OpenloopVelocityActual(addr = 0x22, mode = rw, size=4) {
    OpenloopVelocityActual: u32,
  }

  register OpenloopPhi(addr = 0x23, mode = rw, size=4) {
    OpenloopPhi: u32,
  }

  register UqUdExt(addr = 0x24, mode = rw, size=4) {
    UdExt: u16[0..16],
    UqExt: u16[16..32],
  }

  register AbnDecoderMode(addr = 0x25, mode = rw, size=4) {
    AbnApol: bool[0],
    AbnBpol: bool[1],
    AbnNpol: bool[2],
    AbnUseAbnAsN: bool[3],
    AbnCln: bool[8],
    AbnDirection: bool[12],
  }

  register AbnDecoderPpr(addr = 0x26, mode = rw, size=4) {
    AbnDecoderPpr: u32,
  }

  register AbnDecoderCount(addr = 0x27, mode = rw, size=4) {
    AbnDecoderCount: u32,
  }

  register AbnDecoderCountN(addr = 0x28, mode = rw, size=4) {
    AbnDecoderCountN: u32,
  }

  register AbnDecoderPhiEPhiMOffset(addr = 0x29, mode = rw, size=4) {
    AbnDecoderPhiMOffset: u16[0..16],
    AbnDecoderPhiEOffset: u16[16..32],
  }

  register AbnDecoderPhiEPhiM(addr = 0x2a, mode = rw, size=4) {
    AbnDecoderPhiM: u16[0..16],
    AbnDecoderPhiE: u16[16..32],
  }

  register Abn2DecoderMode(addr = 0x2c, mode = rw, size=4) {
    Abn2Apol: bool[0],
    Abn2Bpol: bool[1],
    Abn2Npol: bool[2],
    Abn2UseAbnAsN: bool[3],
    Abn2Cln: bool[8],
    Abn2Direction: bool[12],
  }

  register Abn2DecoderPpr(addr = 0x2d, mode = rw, size=4) {
    Abn2DecoderPpr: u32,
  }

  register Abn2DecoderCount(addr = 0x2e, mode = rw, size=4) {
    Abn2DecoderCount: u32,
  }

  register Abn2DecoderCountN(addr = 0x2f, mode = rw, size=4) {
    Abn2DecoderCountN: u32,
  }

  register Abn2DecoderPhiMOffset(addr = 0x30, mode = rw, size=4) {
    Abn2DecoderPhiMOffset: u32,
  }

  register Abn2DecoderPhiM(addr = 0x31, mode = rw, size=4) {
    Abn2DecoderPhiM: u32,
  }

  register HallMode(addr = 0x33, mode = rw, size=4) {
    HallPolarity: bool[0],
    HallSync: bool[4],
    HallInterp: bool[8],
    HallDir: bool[12],
    HallBlank: u16[16..28],
  }

  register HallPosition060000(addr = 0x34, mode = rw, size=4) {
    HallPosition000: u16[0..16],
    HallPosition060: u16[16..32],
  }

  register HallPosition180120(addr = 0x35, mode = rw, size=4) {
    HallPosition120: u16[0..16],
    HallPosition180: u16[16..32],
  }

  register HallPosition300240(addr = 0x36, mode = rw, size=4) {
    HallPosition240: u16[0..16],
    HallPosition300: u16[16..32],
  }

  register HallPhiEPhiMOffset(addr = 0x37, mode = rw, size=4) {
    HallPhiMOffset: u16[0..16],
    HallPhiEOffset: u16[16..32],
  }

  register HallDphiMax(addr = 0x38, mode = rw, size=4) {
    HallDphiMax: u32,
  }

  register HallPhiEInterpolatedPhiE(addr = 0x39, mode = rw, size=4) {
    HallPhiE: u16[0..16],
    HallPhiEInterpolated: u16[16..32],
  }

  register HallPhiM(addr = 0x3a, mode = rw, size=4) {
    HallPhiM: u32,
  }

  register AencDecoderMode(addr = 0x3b, mode = rw, size=4) {
    AencDeg: bool[0],
    AencDir: bool[12],
  }

  register AencDecoderNThreshold(addr = 0x3c, mode = rw, size=4) {
    AencDecoderNThreshold: u32,
  }

  register AencDecoderPhiARaw(addr = 0x3d, mode = rw, size=4) {
    AencDecoderPhiARaw: u32,
  }

  register AencDecoderPhiAOffset(addr = 0x3e, mode = rw, size=4) {
    AencDecoderPhiAOffset: u32,
  }

  register AencDecoderPhiA(addr = 0x3f, mode = rw, size=4) {
    AencDecoderPhiA: u32,
  }

  register AencDecoderPpr(addr = 0x40, mode = rw, size=4) {
    AencPpr: u16[0..16],
  }

  register AencDecoderCount(addr = 0x41, mode = rw, size=4) {
    AencDecoderCount: u32,
  }

  register AencDecoderCountN(addr = 0x42, mode = rw, size=4) {
    AencDecoderCountN: u32,
  }

  register AencDecoderPhiEPhiMOffset(addr = 0x45, mode = rw, size=4) {
    AencDecoderPhiMOffset: u16[0..16],
    AencDecoderPhiEOffset: u16[16..32],
  }

  register AencDecoderPhiEPhiM(addr = 0x46, mode = rw, size=4) {
    AencDecoderPhiM: u16[0..16],
    AencDecoderPhiE: u16[16..32],
  }

  register ConfigData(addr = 0x4d, mode = rw, size=4) {
    ConfigData: u32,
  }

  enum ConfigAddr: u32{32} {
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
  }

  register ConfigAddr(addr = 0x4e, mode = rw, size=4) {
    ConfigAddr: u32,
  }

  register ConfigBiquadXA1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXA1: u32,
  }

  register ConfigBiquadXA2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXA2: u32,
  }

  register ConfigBiquadXB0(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXB0: u32,
  }

  register ConfigBiquadXB1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXB1: u32,
  }

  register ConfigBiquadXB2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXB2: u32,
  }

  register ConfigBiquadXEnable(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadXEnable: u32,
  }

  register ConfigBiquadVA1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVA1: u32,
  }

  register ConfigBiquadVA2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVA2: u32,
  }

  register ConfigBiquadVB0(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVB0: u32,
  }

  register ConfigBiquadVB1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVB1: u32,
  }

  register ConfigBiquadVB2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVB2: u32,
  }

  register ConfigBiquadVEnable(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadVEnable: u32,
  }

  register ConfigBiquadTA1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTA1: u32,
  }

  register ConfigBiquadTA2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTA2: u32,
  }

  register ConfigBiquadTB0(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTB0: u32,
  }

  register ConfigBiquadTB1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTB1: u32,
  }

  register ConfigBiquadTB2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTB2: u32,
  }

  register ConfigBiquadTEnable(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadTEnable: u32,
  }

  register ConfigBiquadFA1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFA1: u32,
  }

  register ConfigBiquadFA2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFA2: u32,
  }

  register ConfigBiquadFB0(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFB0: u32,
  }

  register ConfigBiquadFB1(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFB1: u32,
  }

  register ConfigBiquadFB2(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFB2: u32,
  }

  register ConfigBiquadFEnable(addr = 0x4d, mode = rw, size=4) {
    ConfigBiquadFEnable: u32,
  }

  register FeedForwardVelocityGain(addr = 0x4d, mode = rw, size=4) {
    FeedForwardVelocityGain: u32,
  }

  register FeedForwardVelocityFilterConstant(addr = 0x4d, mode = rw, size=4) {
    FeedForwardVelocityFilterConstant: u32,
  }

  register FeedForwardTorqueGain(addr = 0x4d, mode = rw, size=4) {
    FeedForwardTorqueGain: u32,
  }

  register FeedForwardTorqueFilterConstant(addr = 0x4d, mode = rw, size=4) {
    FeedForwardTorqueFilterConstant: u32,
  }

  register ConfigRefSwitchConfig(addr = 0x4d, mode = rw, size=4) {
    ConfigRefSwitchConfig: u32,
  }

  register ConfigSinglePinIfStatusCfg(addr = 0x4d, mode = rw, size=4) {
    ConfigSinglePinIfStatusCfg: u32,
  }

  register ConfigSinglePinIfScaleOffset(addr = 0x4d, mode = rw, size=4) {
    ConfigSinglePinIfScaleOffset: u32,
  }

  register ConfigAdvancedPiRepresent(addr = 0x4d, mode = rw, size=4) {
    CurrentIN: bool[0],
    CurrentPN: bool[1],
    VelocityIN: bool[2],
    VelocityPN: bool[3],
    PositionIN: bool[4],
    PositionPN: bool[5],
  }

  register VelocitySelection(addr = 0x50, mode = rw, size=4) {
    VelocitySelection: u8[0..8],
    VelocityMeterSelection: u8[8..16],
  }

  register PositionSelection(addr = 0x51, mode = rw, size=4) {
    PositionSelection: u32,
  }

  register PhiESelection(addr = 0x52, mode = rw, size=4) {
    PhiESelection: u32,
  }

  register PhiE(addr = 0x53, mode = rw, size=4) {
    PhiE: u32,
  }

  register PidFluxPFluxI(addr = 0x54, mode = rw, size=4) {
    PidFluxI: u16[0..16],
    PidFluxP: u16[16..32],
  }

  register PidTorquePTorqueI(addr = 0x56, mode = rw, size=4) {
    PidTorqueI: u16[0..16],
    PidTorqueP: u16[16..32],
  }

  register PidVelocityPVelocityI(addr = 0x58, mode = rw, size=4) {
    PidVelocityI: u16[0..16],
    PidVelocityP: u16[16..32],
  }

  register PidPositionPPositionI(addr = 0x5a, mode = rw, size=4) {
    PidPositionI: u16[0..16],
    PidPositionP: u16[16..32],
  }

  register PidoutUqUdLimits(addr = 0x5d, mode = rw, size=4) {
    PidoutUqUdLimits: u32,
  }

  register PidTorqueFluxLimits(addr = 0x5e, mode = rw, size=4) {
    PidTorqueFluxLimits: u32,
  }

  register PidVelocityLimit(addr = 0x60, mode = rw, size=4) {
    PidVelocityLimit: u32,
  }

  register PidPositionLimitLow(addr = 0x61, mode = rw, size=4) {
    PidPositionLimitLow: u32,
  }

  register PidPositionLimitHigh(addr = 0x62, mode = rw, size=4) {
    PidPositionLimitHigh: u32,
  }

  register ModeRampModeMotion(addr = 0x63, mode = rw, size=4) {
    ModeMotion: u8[0..8],
    ModePidSmpl: u8[24..31],
    ModeFf: u8[16..24],
    ModePidType: bool[31],
  }

  register PidTorqueFluxTarget(addr = 0x64, mode = rw, size=4) {
    PidFluxTarget: u16[0..16],
    PidTorqueTarget: u16[16..32],
  }

  register PidTorqueFluxOffset(addr = 0x65, mode = rw, size=4) {
    PidFluxOffset: u16[0..16],
    PidTorqueOffset: u16[16..32],
  }

  register PidVelocityTarget(addr = 0x66, mode = rw, size=4) {
    PidVelocityTarget: u32,
  }

  register PidVelocityOffset(addr = 0x67, mode = rw, size=4) {
    PidVelocityOffset: u32,
  }

  register PidPositionTarget(addr = 0x68, mode = rw, size=4) {
    PidPositionTarget: u32,
  }

  register PidTorqueFluxActual(addr = 0x69, mode = rw, size=4) {
    PidFluxActual: u16[0..16],
    PidTorqueActual: u16[16..32],
  }

  register PidVelocityActual(addr = 0x6a, mode = rw, size=4) {
    PidVelocityActual: u32,
  }

  register PidPositionActual(addr = 0x6b, mode = rw, size=4) {
    PidPositionActual: u32,
  }

  register PidErrorData(addr = 0x6c, mode = rw, size=4) {
    PidErrorData: u32,
  }

  enum PidErrorAddr: u32{32} {
    0 PidErrorPidTorqueError,
    1 PidErrorPidFluxError,
    2 PidErrorPidVelocityError,
    3 PidErrorPidPositionError,
    4 PidErrorPidTorqueErrorSum,
    5 PidErrorPidFluxErrorSum,
    6 PidErrorPidVelocityErrorSum,
    7 PidErrorPidPositionErrorSum,
  }

  register PidErrorAddr(addr = 0x6d, mode = rw, size=4) {
    PidErrorAddr: u32,
  }

  register PidErrorPidTorqueError(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidTorqueError: u32,
  }

  register PidErrorPidFluxError(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidFluxError: u32,
  }

  register PidErrorPidVelocityError(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidVelocityError: u32,
  }

  register PidErrorPidPositionError(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidPositionError: u32,
  }

  register PidErrorPidTorqueErrorSum(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidTorqueErrorSum: u32,
  }

  register PidErrorPidFluxErrorSum(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidFluxErrorSum: u32,
  }

  register PidErrorPidVelocityErrorSum(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidVelocityErrorSum: u32,
  }

  register PidErrorPidPositionErrorSum(addr = 0x6c, mode = rw, size=4) {
    PidErrorPidPositionErrorSum: u32,
  }

  register InterimData(addr = 0x6e, mode = rw, size=4) {
    InterimData: u32,
  }

  enum InterimAddr: u32{32} {
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
  }

  register InterimAddr(addr = 0x6f, mode = rw, size=4) {
    InterimAddr: u32,
  }

  register InterimPidinTargetTorque(addr = 0x6e, mode = rw, size=4) {
    InterimPidinTargetTorque: u32,
  }

  register InterimPidinTargetFlux(addr = 0x6e, mode = rw, size=4) {
    InterimPidinTargetFlux: u32,
  }

  register InterimPidinTargetVelocity(addr = 0x6e, mode = rw, size=4) {
    InterimPidinTargetVelocity: u32,
  }

  register InterimPidinTargetPosition(addr = 0x6e, mode = rw, size=4) {
    InterimPidinTargetPosition: u32,
  }

  register InterimPidoutTargetTorque(addr = 0x6e, mode = rw, size=4) {
    InterimPidoutTargetTorque: u32,
  }

  register InterimPidoutTargetFlux(addr = 0x6e, mode = rw, size=4) {
    InterimPidoutTargetFlux: u32,
  }

  register InterimPidoutTargetVelocity(addr = 0x6e, mode = rw, size=4) {
    InterimPidoutTargetVelocity: u32,
  }

  register InterimPidoutTargetPosition(addr = 0x6e, mode = rw, size=4) {
    InterimPidoutTargetPosition: u32,
  }

  register InterimFocIwyIux(addr = 0x6e, mode = rw, size=4) {
    InterimFocIwyIux: u32,
  }

  register InterimFocIv(addr = 0x6e, mode = rw, size=4) {
    InterimFocIv: u32,
  }

  register InterimFocIbIa(addr = 0x6e, mode = rw, size=4) {
    InterimFocIbIa: u32,
  }

  register InterimFocIqId(addr = 0x6e, mode = rw, size=4) {
    InterimFocIqId: u32,
  }

  register InterimFocUqUd(addr = 0x6e, mode = rw, size=4) {
    InterimFocUqUd: u32,
  }

  register InterimFocUqUdLimited(addr = 0x6e, mode = rw, size=4) {
    InterimFocUqUdLimited: u32,
  }

  register InterimFocUbUa(addr = 0x6e, mode = rw, size=4) {
    InterimFocUbUa: u32,
  }

  register InterimFocUwyUux(addr = 0x6e, mode = rw, size=4) {
    InterimFocUwyUux: u32,
  }

  register InterimFocUv(addr = 0x6e, mode = rw, size=4) {
    InterimFocUv: u32,
  }

  register InterimPwmWyUx(addr = 0x6e, mode = rw, size=4) {
    InterimPwmUx: u16[0..16],
    InterimPwmWy: u16[16..32],
  }

  register InterimPwmUv(addr = 0x6e, mode = rw, size=4) {
    InterimPwmUv: u32,
  }

  register InterimAdcI1I0(addr = 0x6e, mode = rw, size=4) {
    InterimAdcI1I0: u32,
  }

  register InterimPidTorqueTargetFluxTargetTorqueActualFluxActualDiv256(addr = 0x6e, mode = rw, size=4) {
    InterimPidTorqueTargetFluxTargetTorqueActualFluxActualDiv256: u32,
  }

  register InterimPidTorqueTargetTorqueActual(addr = 0x6e, mode = rw, size=4) {
    InterimPidTorqueTargetTorqueActual: u32,
  }

  register InterimPidFluxTargetFluxActual(addr = 0x6e, mode = rw, size=4) {
    InterimPidFluxTargetFluxActual: u32,
  }

  register InterimPidVelocityTargetVelocityActualDiv256(addr = 0x6e, mode = rw, size=4) {
    InterimPidVelocityTargetVelocityActualDiv256: u32,
  }

  register InterimPidVelocityTargetVelocityActual(addr = 0x6e, mode = rw, size=4) {
    InterimPidVelocityTargetVelocityActual: u32,
  }

  register InterimPidPositionTargetPositionActualDiv256(addr = 0x6e, mode = rw, size=4) {
    InterimPidPositionTargetPositionActualDiv256: u32,
  }

  register InterimPidPositionTargetPositionActual(addr = 0x6e, mode = rw, size=4) {
    InterimPidPositionTargetPositionActual: u32,
  }

  register InterimFfVelocity(addr = 0x6e, mode = rw, size=4) {
    InterimFfVelocity: u32,
  }

  register InterimFfTorque(addr = 0x6e, mode = rw, size=4) {
    InterimFfTorque: u32,
  }

  register InterimActualVelocityPptm(addr = 0x6e, mode = rw, size=4) {
    InterimActualVelocityPptm: u32,
  }

  register InterimRefSwitchStatus(addr = 0x6e, mode = rw, size=4) {
    InterimRefSwitchStatus: u32,
  }

  register InterimHomePosition(addr = 0x6e, mode = rw, size=4) {
    InterimHomePosition: u32,
  }

  register InterimLeftPosition(addr = 0x6e, mode = rw, size=4) {
    InterimLeftPosition: u32,
  }

  register InterimRightPosition(addr = 0x6e, mode = rw, size=4) {
    InterimRightPosition: u32,
  }

  register InterimSinglePinIfPwmDutyCycleTorqueTarget(addr = 0x6e, mode = rw, size=4) {
    InterimSinglePinIfPwmDutyCycleTorqueTarget: u32,
  }

  register InterimSinglePinIfVelocityTarget(addr = 0x6e, mode = rw, size=4) {
    InterimSinglePinIfVelocityTarget: u32,
  }

  register InterimSinglePinIfPositionTarget(addr = 0x6e, mode = rw, size=4) {
    InterimSinglePinIfPositionTarget: u32,
  }

  register AdcVmLimits(addr = 0x75, mode = rw, size=4) {
    AdcVmLimitLow: u16[0..16],
    AdcVmLimitHigh: u16[16..32],
  }

  register Tmc4671InputsRaw(addr = 0x76, mode = rw, size=4) {
    Tmc4671InputsRaw: u32,
  }

  register Tmc4671OutputsRaw(addr = 0x77, mode = rw, size=4) {
    Tmc4671OutputsRaw: u32,
  }

  register StepWidth(addr = 0x78, mode = rw, size=4) {
    StepWidth: u32,
  }

  register UartBps(addr = 0x79, mode = rw, size=4) {
    UartBps: u32,
  }

  register GpioDsadciConfig(addr = 0x7b, mode = rw, size=4) {
    GpioDsadciConfig: u32,
  }

  register StatusFlags(addr = 0x7c, mode = rw, size=4) {
    PidXTargetLimit: bool[0],
    PidXErrsumLimit: bool[2],
    PidXOutputLimit: bool[3],
    PidVTargetLimit: bool[4],
    PidVErrsumLimit: bool[6],
    PidVOutputLimit: bool[7],
    PidIdTargetLimit: bool[8],
    PidIdErrsumLimit: bool[10],
    PidIdOutputLimit: bool[11],
    PidIqTargetLimit: bool[12],
    PidIqErrsumLimit: bool[14],
    PidIqOutputLimit: bool[15],
    IparkCirlimLimitUD: bool[16],
    IparkCirlimLimitUQ: bool[17],
    IparkCirlimLimitUR: bool[18],
    RefSwR: bool[20],
    RefSwH: bool[21],
    RefSwL: bool[22],
    PwmMin: bool[24],
    PwmMax: bool[25],
    AdcIClipped: bool[26],
    AencClipped: bool[27],
    EncN: bool[28],
    Enc2N: bool[29],
    AencN: bool[30],
  }

  register StatusMask(addr = 0x7d, mode = rw, size=4) {
    PidXTargetLimit: bool[0],
    PidXErrsumLimit: bool[2],
    PidXOutputLimit: bool[3],
    PidVTargetLimit: bool[4],
    PidVErrsumLimit: bool[6],
    PidVOutputLimit: bool[7],
    PidIdTargetLimit: bool[8],
    PidIdErrsumLimit: bool[10],
    PidIdOutputLimit: bool[11],
    PidIqTargetLimit: bool[12],
    PidIqErrsumLimit: bool[14],
    PidIqOutputLimit: bool[15],
    IparkCirlimLimitUD: bool[16],
    IparkCirlimLimitUQ: bool[17],
    IparkCirlimLimitUR: bool[18],
    RefSwR: bool[20],
    RefSwH: bool[21],
    RefSwL: bool[22],
    PwmMin: bool[24],
    PwmMax: bool[25],
    AdcIClipped: bool[26],
    AencClipped: bool[27],
    EncN: bool[28],
    Enc2N: bool[29],
    AencN: bool[30],
  }

}
