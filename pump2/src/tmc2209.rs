use deku::prelude::*;
use deku::ctx::BitSize;
use esp_idf_hal::{
    delay::TickType,
    gpio::{AnyIOPin, InputPin, Output, OutputPin, PinDriver},
    peripheral::Peripheral,
    uart::{
        config::{Config, DataBits, StopBits},
        Uart, UartDriver,
    },
    units::Hertz,
};
use std::{
    fmt,
    io::Cursor,
    time::Duration,
};

// General configuration registers.
const GLOBAL_CONFIG_REG: u8 = 0x00;
const GLOBAL_STATUS_REG: u8 = 0x01;
const TRANSMISSION_COUNT_REG: u8 = 0x02;
const RESPONSE_DELAY_REG: u8 = 0x03;
const DEFAULTS_WRITE_REG: u8 = 0x04;
const DEFAULTS_READ_REG: u8 = 0x05;
const PIN_STATES_REG: u8 = 0x06;
const FACTORY_CONFIG_REG: u8 = 0x07;
// Velocity dependent control.
const CURRENT_CONFIG_REG: u8 = 0x10;
const POWER_DOWN_DELAY_REG: u8 = 0x11;
const MICROSTEP_TIME_REG: u8 = 0x12;
const PWM_THRESHOLD_REG: u8 = 0x13;
const VELOCITY_REG: u8 = 0x22;
const COOLSTEP_THRESHOLD_REG: u8 = 0x14;
const STALLGUARD_THRESHOLD_REG: u8 = 0x40;
const MOTOR_LOAD_REG: u8 = 0x41;
const COOLSTEP_CONFIG_REG: u8 = 0x42;
// Microstepping control registers.
const MICROSTEP_POSITION_REG: u8 = 0x6A;
const MICROSTEP_CURRENT_REG: u8 = 0x6B;
// Driver control registers.
const DRIVER_CONFIG_REG: u8 = 0x6C;
const DRIVER_STATUS_REG: u8 = 0x6F;
const PWM_CONFIG_REG: u8 = 0x70;
const PWM_SCALE_REG: u8 = 0x71;
const PWM_AUTO_REG: u8 = 0x72;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "big", magic = b"\x05")]
struct ReadRequestFrame {
    address: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 7)]
    register: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "big", magic = b"\x05\xff")]
struct ReadResponseFrame {
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 7)]
    register: u8,
    #[deku(ctx = "*register")]
    data: FrameData,
    crc: u8,
}

#[deku_derive(DekuRead, DekuWrite)]
#[derive(Debug, Clone, Copy)]
#[deku(endian = "big", magic = b"\x05")]
struct WriteFrame {
    address: u8,
    #[deku(bits = 1, temp, temp_value = "true")]
    write_bit: bool,
    #[deku(bits = 7)]
    register: u8,
    #[deku(ctx = "*register")]
    data: FrameData,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian, register: u8", id = "register", endian = "endian")]
enum FrameData {
    // General configuration registers.
    #[deku(id = "GLOBAL_CONFIG_REG")]
    GlobalConfig(GlobalConfig),

    #[deku(id = "GLOBAL_STATUS_REG")]
    GlobalStatus(GlobalStatus),

    #[deku(id = "TRANSMISSION_COUNT_REG")]
    TransmissionCount(
        #[deku(pad_bits_before = "24")]
        u8
    ),

    #[deku(id = "RESPONSE_DELAY_REG")]
    ResponseDelay(
        #[deku(pad_bits_before = "20")]
        #[deku(bits = "4")]
        #[deku(pad_bits_after = "8")]
        u8
    ),

    #[deku(id = "DEFAULTS_WRITE_REG")]
    DefaultsWrite(DefaultsWrite),

    // TODO: Turn this into a struct.
    #[deku(id = "DEFAULTS_READ_REG")]
    DefaultsRead(u32),

    #[deku(id = "PIN_STATES_REG")]
    PinStates(PinStates),

    #[deku(id = "FACTORY_CONFIG_REG")]
    FactoryConfig(FactoryConfig),

    // Velocity dependent control.
    #[deku(id = "CURRENT_CONFIG_REG")]
    CurrentConfig(CurrentConfig),

    #[deku(id = "POWER_DOWN_DELAY_REG")]
    PowerDownDelay(
        #[deku(pad_bits_before = "24")]
        u8
    ),

    #[deku(id = "MICROSTEP_TIME_REG")]
    MicrostepTime(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32
    ),

    #[deku(id = "PWM_THRESHOLD_REG")]
    PwmThreshold(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32
    ),

    #[deku(id = "VELOCITY_REG")]
    Velocity(
        #[deku(pad_bits_before = "8")]
        #[deku(bits = 24)]
        i32
    ),

    #[deku(id = "COOLSTEP_THRESHOLD_REG")]
    CoolstepThreshold(
        #[deku(pad_bits_before = "12")]
        #[deku(bits = 20)]
        u32
    ),

    #[deku(id = "STALLGUARD_THRESHOLD_REG")]
    StallguardThreshold(
        #[deku(pad_bits_before = "24")]
        u8
    ),

    #[deku(id = "MOTOR_LOAD_REG")]
    MotorLoad(
        #[deku(pad_bits_before = "22")]
        #[deku(bits = 10)]
        u16
    ),

    #[deku(id = "COOLSTEP_CONFIG_REG")]
    CoolstepConfig(CoolstepConfig),

    // Microstepping control registers.
    #[deku(id = "MICROSTEP_POSITION_REG")]
    MicrostepPosition(
        #[deku(pad_bits_before = "22")]
        #[deku(bits = 10)]
        u16
    ),

    #[deku(id = "MICROSTEP_CURRENT_REG")]
    MicrostepCurrent(
        #[deku(pad_bits_before = "7")]
        #[deku(bits = 9)]
        i16,
        #[deku(pad_bits_before = "7")]
        #[deku(bits = 9)]
        i16
    ),

    // Driver control registers.
    #[deku(id = "DRIVER_CONFIG_REG")]
    DriverConfig(DriverConfig),

    #[deku(id = "DRIVER_STATUS_REG")]
    DriverStatus(DriverStatus),

    #[deku(id = "PWM_CONFIG_REG")]
    PwmConfig(PwmConfig),

    #[deku(id = "PWM_SCALE_REG")]
    PwmScale(PwmScale),

    #[deku(id = "PWM_AUTO_REG")]
    PwmAuto(PwmAuto),
}

// General configuration registers.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct GlobalConfig {
    #[deku(pad_bits_before = "22")]
    #[deku(bits = 1)]
    test_mode: bool,
    #[deku(bits = 1)]
    filter_step_pulses: bool,
    #[deku(bits = 1)]
    uart_selects_microsteps: bool,
    #[deku(bits = 1)]
    pin_uart_mode: bool,
    index_output: IndexOutput,
    #[deku(bits = 1)]
    invert_direction: bool,
    #[deku(bits = 1)]
    disable_pwm: bool,
    #[deku(bits = 1)]
    internal_sense_resistor: bool,
    #[deku(bits = 1)]
    external_current_scaling: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum IndexOutput {
    #[deku(id = 0b00)]
    Period,
    #[deku(id = 0b01)]
    Overtemperature,
    #[deku(id = 0b10)]
    Microstep,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct GlobalStatus {
    #[deku(pad_bits_before = "29")]
    #[deku(bits = 1)]
    charge_pump_undervoltage: bool,
    #[deku(bits = 1)]
    driver_error: bool,
    #[deku(bits = 1)]
    reset: bool,
}

#[deku_derive(DekuRead, DekuWrite)]
#[derive(Debug, Clone, Copy)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DefaultsWrite {
    #[deku(pad_bits_before = "16")]
    #[deku(temp, temp_value = "0xBD")]
    magic: u8,
    #[deku(pad_bits_before = "2")]
    #[deku(bits = 2)]
    byte_address: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 3)]
    bit_address: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PinStates {
    version: u8,
    #[deku(pad_bits_before = "14")]
    #[deku(bits = 1)]
    direction: bool,
    #[deku(bits = 1)]
    disable_pwm: bool,
    #[deku(bits = 1)]
    step: bool,
    #[deku(bits = 1)]
    powerdown_uart: bool,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 1)]
    diagnostic: bool,
    #[deku(bits = 1)]
    microstep2: bool,
    #[deku(bits = 1)]
    microstep1: bool,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 1)]
    disable: bool,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct FactoryConfig {
    #[deku(pad_bits_before = "22")]
    #[deku(bits = 2)]
    overtemperature_threshold: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    clock_frequency: u8,
}

// Velocity dependent control.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct CurrentConfig {
    #[deku(pad_bits_before = "12")]
    #[deku(bits = 4)]
    powerdown_time: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    running_current_scale: u8,
    #[deku(pad_bits_before = "3")]
    #[deku(bits = 5)]
    stopped_current_scale: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct CoolstepConfig {
    #[deku(pad_bits_before = "16")]
    #[deku(bits = 1)]
    lower_min_current: bool,
    #[deku(bits = 2)]
    current_downstep_rate: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 4)]
    stallguard_hysteresis: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 2)]
    current_upstep: u8,
    #[deku(pad_bits_before = "1")]
    #[deku(bits = 4)]
    stallguard_threshold: u8,
}

// Driver control registers.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DriverConfig {
    #[deku(bits = 1)]
    disable_short_supply_protect: bool,
    #[deku(bits = 1)]
    disable_short_ground_protect: bool,
    #[deku(bits = 1)]
    double_edge_step: bool,
    #[deku(bits = 1)]
    interpolate_microsteps: bool,
    microstep_resolution: MicrostepResolution,
    #[deku(pad_bits_before = "6")]
    #[deku(bits = 1)]
    low_sense_resistor_voltage: bool,
    blank_time: BlankTime,
    #[deku(pad_bits_before = "4")]
    #[deku(map = "|x: i8| -> std::result::Result<_, DekuError> { Ok(((x & 0x0F) as u8 as i8) - 3)}")]
    #[deku(writer = "(self.hysteresis_end + 3).to_writer(deku::writer, BitSize(4))")]
    #[deku(bits = 4)]
    hysteresis_end: i8,
    #[deku(map = "|x: u8| -> std::result::Result<_, DekuError> { Ok(x + 1)}")]
    #[deku(writer = "(self.hysteresis_start - 1).to_writer(deku::writer, BitSize(3))")]
    #[deku(bits = 3)]
    hysteresis_start: u8,
    #[deku(bits = 4)]
    decay_time: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 4, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum MicrostepResolution {
    #[deku(id = 0b0000)]
    M256,
    #[deku(id = 0b0001)]
    M128,
    #[deku(id = 0b0010)]
    M64,
    #[deku(id = 0b0011)]
    M32,
    #[deku(id = 0b0100)]
    M16,
    #[deku(id = 0b0101)]
    M8,
    #[deku(id = 0b0110)]
    M4,
    #[deku(id = 0b0111)]
    M2,
    #[deku(id = 0b1000)]
    M1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum BlankTime {
    #[deku(id = 0b00)]
    B16,
    #[deku(id = 0b01)]
    B24,
    #[deku(id = 0b10)]
    B36,
    #[deku(id = 0b11)]
    B54,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct DriverStatus {
    #[deku(bits = 1)]
    stopped: bool,
    #[deku(bits = 1)]
    pwm_mode: bool,
    #[deku(pad_bits_before = "9")]
    #[deku(bits = 5)]
    current_scale: u8,
    #[deku(pad_bits_before = "4")]
    temperature: TemperatureThreshold,
    open_load: PhaseStatus,
    low_side_short: PhaseStatus,
    ground_short: PhaseStatus,
    overtemperature: OvertemperatureStatus,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum PhaseStatus {
    #[deku(id = 0b00)]
    None,
    #[deku(id = 0b01)]
    PhaseA,
    #[deku(id = 0b10)]
    PhaseB,
    #[deku(id = 0b11)]
    BothPhases,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 4, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum TemperatureThreshold {
    #[deku(id = 0b0000)]
    Normal,
    #[deku(id = 0b0001)]
    Temp120C,
    #[deku(id = 0b0011)]
    Temp143C,
    #[deku(id = 0b0111)]
    Temp150C,
    #[deku(id = 0b1111)]
    Temp157C,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum OvertemperatureStatus {
    #[deku(id = 0b00)]
    Normal,
    #[deku(id = 0b01)]
    Warning,
    #[deku(id = 0b11)]
    Shutdown,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmConfig {
    #[deku(bits = 4)]
    driver_switch_autoscale_limit: u8,
    #[deku(bits = 4)]
    max_amplitude_change: u8,
    #[deku(pad_bits_before = "2")]
    freewheel_mode: FreewheelMode,
    #[deku(bits = 1)]
    autogradient: bool,
    #[deku(bits = 1)]
    autoscale: bool,
    frequency: PwmFrequency,
    gradient: u8,
    offset: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum PwmFrequency {
    #[deku(id = 0b00)]
    Div1024,
    #[deku(id = 0b01)]
    Div683,
    #[deku(id = 0b10)]
    Div512,
    #[deku(id = 0b11)]
    Div410,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, DekuRead, DekuWrite)]
#[deku(id_type = "u8", bits = 2, ctx = "endian: deku::ctx::Endian", endian = "endian")]
pub enum FreewheelMode {
    #[deku(id = 0b00)]
    Normal,
    #[deku(id = 0b01)]
    Freewheeling,
    #[deku(id = 0b10)]
    ShortLowSide,
    #[deku(id = 0b11)]
    ShortHighSide,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmScale {
    #[deku(pad_bits_before = "7")]
    #[deku(bits = 9)]
    offset: i16,
    #[deku(pad_bits_before = "8")]
    scale: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct PwmAuto {
    #[deku(pad_bits_before = "8")]
    gradient: u8,
    #[deku(pad_bits_before = "8")]
    offset: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct PwmState {
    pub auto_grad: u8,
    pub auto_offset: u8,
    pub scale: u8,
    pub offset: i16,
}

#[derive(Debug)]
pub enum Tmc2209Error {
    UartError(esp_idf_svc::sys::EspError),
    CrcMismatch(String),
    InvalidResponse(String),
    Timeout(String),
}

impl fmt::Display for Tmc2209Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Tmc2209Error::UartError(e) => write!(f, "UART error: {e}"),
            Tmc2209Error::CrcMismatch(e) => write!(f, "CRC mismatch: {e}"),
            Tmc2209Error::InvalidResponse(e) => write!(f, "Invalid response: {e}"),
            Tmc2209Error::Timeout(e) => write!(f, "Timeout: {e}"),
        }
    }
}

impl std::error::Error for Tmc2209Error {}

impl From<esp_idf_svc::sys::EspError> for Tmc2209Error {
    fn from(e: esp_idf_svc::sys::EspError) -> Self {
        Tmc2209Error::UartError(e)
    }
}

impl From<deku::DekuError> for Tmc2209Error {
    fn from(e: deku::DekuError) -> Self {
        Tmc2209Error::InvalidResponse(format!("Deku error: {e}"))
    }
}

pub type Result<T> = std::result::Result<T, Tmc2209Error>;

pub struct Tmc2209<'a, T: OutputPin, R: OutputPin, E: OutputPin> {
    uart: UartDriver<'a>,
    step_pin: PinDriver<'a, T, Output>,
    dir_pin: PinDriver<'a, R, Output>,
    disable_pin: PinDriver<'a, E, Output>,
    address: u8,
    timeout: TickType,
    sense_ohms: f64,
    steps_per_rev: f64,
    global_config: GlobalConfig,
    resp_delay: u8,
    current_config: CurrentConfig,
    pwrdown_delay: u8,
    pwm_thresh: u32,
    vel: i32,
    coolstep_thresh: u32,
    stallguard_thresh: u8,
    coolstep_config: CoolstepConfig,
    driver_config: DriverConfig,
    pwm_config: PwmConfig,
}

impl<'a, STEP: OutputPin, DIR: OutputPin, EN: OutputPin> Tmc2209<'a, STEP, DIR, EN> {
    pub fn new(
        uart: impl Peripheral<P = impl Uart> + 'a,
        tx: impl Peripheral<P = impl OutputPin> + 'a,
        rx: impl Peripheral<P = impl InputPin> + 'a,
        step: STEP,
        dir: DIR,
        disable: EN,
    ) -> Result<Self> {
        let config = Config::new()
            .baudrate(Hertz(115_200))
            .parity_none()
            .data_bits(DataBits::DataBits8)
            .stop_bits(StopBits::STOP1);

        let uart = UartDriver::new(
            uart,
            tx,
            rx,
            Option::<AnyIOPin>::None,
            Option::<AnyIOPin>::None,
            &config,
        )?;

        let step_pin = PinDriver::output(step)?;
        let dir_pin = PinDriver::output(dir)?;
        let disable_pin = PinDriver::output(disable)?;

        let mut tmc = Self {
            uart: uart,
            step_pin: step_pin,
            dir_pin: dir_pin,
            disable_pin: disable_pin,
            address: 0,
            timeout: TickType::from(Duration::from_millis(100)),
            sense_ohms: 0.110,
            steps_per_rev: 200.0,
            global_config: GlobalConfig {
                test_mode: false,
                filter_step_pulses: true,
                uart_selects_microsteps: true,
                pin_uart_mode: true,
                index_output: IndexOutput::Period,
                invert_direction: false,
                disable_pwm: false,
                internal_sense_resistor: false,
                external_current_scaling: false,
            },
            resp_delay: 3,
            current_config: CurrentConfig {
                stopped_current_scale: 10,
                running_current_scale: 0,
                powerdown_time: 8,
            },
            pwrdown_delay: 20,
            pwm_thresh: 999999,
            vel: 0,
            coolstep_thresh: 0,
            stallguard_thresh: 0,
            coolstep_config: CoolstepConfig {
                lower_min_current: false,
                current_downstep_rate: 0,
                stallguard_hysteresis: 0,
                current_upstep: 0,
                stallguard_threshold: 0,
            },
            driver_config: DriverConfig {
                disable_short_supply_protect: false,
                disable_short_ground_protect: false,
                double_edge_step: false,
                interpolate_microsteps: true,
                microstep_resolution: MicrostepResolution::M16,
                low_sense_resistor_voltage: false,
                blank_time: BlankTime::B36,
                hysteresis_end: -3,
                hysteresis_start: 4,
                decay_time: 5,
            },
            pwm_config: PwmConfig {
                driver_switch_autoscale_limit: 12,
                max_amplitude_change: 8,
                freewheel_mode: FreewheelMode::Normal,
                autogradient: true,
                autoscale: true,
                frequency: PwmFrequency::Div683,
                gradient: 14,
                offset: 36,
            },
        };
        tmc.running_amps(0.150);
        Ok(tmc)
    }

    pub fn enable(&mut self) -> Result<()> {
        self.disable_pin.set_low()?;
        Ok(())
    }

    pub fn disable(&mut self) -> Result<()> {
        self.disable_pin.set_high()?;
        Ok(())
    }

    pub fn init(&mut self) -> Result<()> {
        self.write_register(GLOBAL_CONFIG_REG, FrameData::GlobalConfig(self.global_config))?;
        // self.write_register(RESPONSE_DELAY_REG, FrameData::ResponseDelay(self.resp_delay))?;
        // self.write_register(CURRENT_CONFIG_REG, FrameData::CurrentConfig(self.current_config))?;
        // self.write_register(POWER_DOWN_DELAY_REG, FrameData::PowerDownDelay(self.pwrdown_delay))?;
        // self.write_register(PWM_THRESHOLD_REG, FrameData::PwmThreshold(self.pwm_thresh))?;
        // self.write_register(VELOCITY_REG, FrameData::Velocity(self.vel))?;
        // self.write_register(COOLSTEP_THRESHOLD_REG, FrameData::CoolstepThreshold(self.coolstep_thresh))?;
        // self.write_register(STALLGUARD_THRESHOLD_REG, FrameData::StallguardThreshold(self.stallguard_thresh))?;
        // self.write_register(COOLSTEP_CONFIG_REG, FrameData::CoolstepConfig(self.coolstep_config))?;
        // self.write_register(DRIVER_CONFIG_REG, FrameData::DriverConfig(self.driver_config))?;
        // self.write_register(PWM_CONFIG_REG, FrameData::PwmConfig(self.pwm_config))?;
        Ok(())
    }

    // =============================
    // ====Configuration options====
    // =============================

    // Global Configs
    pub fn timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = TickType::from(timeout);
        self
    }

    pub fn steps_per_rev(&mut self, steps: f64) -> &mut Self {
        self.steps_per_rev = steps;
        self
    }

    pub fn test_mode(&mut self, enable: bool) -> &mut Self {
        self.global_config.test_mode = enable;
        self
    }

    pub fn filter_step_pulses(&mut self, enable: bool) -> &mut Self {
        self.global_config.filter_step_pulses = enable;
        self
    }

    pub fn uart_selects_microsteps(&mut self, enable: bool) -> &mut Self {
        self.global_config.uart_selects_microsteps = enable;
        self
    }

    pub fn pin_uart_mode(&mut self, enable: bool) -> &mut Self {
        self.global_config.pin_uart_mode = enable;
        self
    }

    pub fn index_output(&mut self, output: IndexOutput) -> &mut Self {
        self.global_config.index_output = output;
        self
    }

    pub fn invert_direction(&mut self, enable: bool) -> &mut Self {
        self.global_config.invert_direction = enable;
        self
    }

    pub fn enable_pwm(&mut self, enable: bool) -> &mut Self {
        self.global_config.disable_pwm = !enable;
        self
    }

    pub fn internal_sense_resistor(&mut self, enable: bool) -> &mut Self {
        self.global_config.internal_sense_resistor = enable;
        self
    }

    pub fn external_current_scaling(&mut self, enable: bool) -> &mut Self {
        self.global_config.external_current_scaling = enable;
        self
    }

    pub fn response_delay(&mut self, delay: u8) -> &mut Self {
        assert!((0..=15).contains(&delay), "Response delay must be between 0 and 15.");
        self.resp_delay = delay;
        self
    }

    // Current Configs
    pub fn stopped_amps(&mut self, rms_amps: f64) -> &mut Self {
        self.current_config.stopped_current_scale = self.get_current_scale(rms_amps);
        self
    }

    pub fn running_amps(&mut self, rms_amps: f64) -> &mut Self {
        self.current_config.running_current_scale = self.get_current_scale(rms_amps);
        self
    }

    pub fn powerdown_time(&mut self, time: u8) -> &mut Self {
        self.current_config.powerdown_time = time;
        self
    }

    pub fn powerdown_delay(&mut self, delay: u8) -> &mut Self {
        self.pwrdown_delay = delay;
        self
    }

    pub fn pwm_threshold(&mut self, threshold: u32) -> &mut Self {
        self.pwm_thresh = threshold;
        self
    }

    pub fn velocity(&mut self, velocity: i32) -> &mut Self {
        self.vel = velocity;
        self
    }

    // Stallguard and coolstep control
    pub fn coolstep_threshold(&mut self, threshold: u32) -> &mut Self {
        self.coolstep_thresh = threshold;
        self
    }

    pub fn stallguard_threshold(&mut self, threshold: u8) -> &mut Self {
        self.stallguard_thresh = threshold;
        self
    }

    pub fn coolstep_lower_min_current(&mut self, enable: bool) -> &mut Self {
        self.coolstep_config.lower_min_current = enable;
        self
    }

    pub fn coolstep_current_downstep_rate(&mut self, rate: u8) -> &mut Self {
        self.coolstep_config.current_downstep_rate = rate;
        self
    }

    pub fn stallguard_hysteresis(&mut self, hysteresis: u8) -> &mut Self {
        self.coolstep_config.stallguard_hysteresis = hysteresis;
        self
    }

    pub fn current_upstep(&mut self, upstep: u8) -> &mut Self {
        self.coolstep_config.current_upstep = upstep;
        self
    }

    pub fn coolstep_stallguard_threshold(&mut self, threshold: u8) -> &mut Self {
        self.coolstep_config.stallguard_threshold = threshold;
        self
    }

    // Driver Configs
    pub fn short_supply_protect(&mut self, enable: bool) -> &mut Self {
        self.driver_config.disable_short_supply_protect = !enable;
        self
    }

    pub fn short_ground_protect(&mut self, enable: bool) -> &mut Self {
        self.driver_config.disable_short_ground_protect = !enable;
        self
    }

    pub fn double_edge_step(&mut self, enable: bool) -> &mut Self {
        self.driver_config.double_edge_step = enable;
        self
    }

    pub fn interpolate_microsteps(&mut self, enable: bool) -> &mut Self {
        self.driver_config.interpolate_microsteps = enable;
        self
    }

    pub fn microstep_resolution(&mut self, resolution: MicrostepResolution) -> &mut Self {
        self.driver_config.microstep_resolution = resolution;
        self
    }

    pub fn low_sense_resistor_voltage(&mut self, enable: bool) -> &mut Self {
        self.driver_config.low_sense_resistor_voltage = enable;
        self
    }

    pub fn blank_time(&mut self, time: BlankTime) -> &mut Self {
        self.driver_config.blank_time = time;
        self
    }

    pub fn hysteresis_end(&mut self, end: i8) -> &mut Self {
        assert!((-3..=12).contains(&end), "Hysteresis end must be between -3 and 12.");
        self.driver_config.hysteresis_end = end;
        self
    }

    pub fn hysteresis_start(&mut self, start: u8) -> &mut Self {
        assert!((0..=7).contains(&start), "Hysteresis start must be between 0 and 7.");
        self.driver_config.hysteresis_start = start;
        self
    }

    pub fn decay_time(&mut self, time: u8) -> &mut Self {
        assert!((0..=15).contains(&time), "Decay time must be between 0 and 15.");
        self.driver_config.decay_time = time;
        self
    }

    // PWM Configs
    pub fn driver_switch_autoscale_limit(&mut self, limit: u8) -> &mut Self {
        assert!((0..=15).contains(&limit), "Autoscale limit mut be between 0 and 15.");
        self.pwm_config.driver_switch_autoscale_limit = limit;
        self
    }

    pub fn max_amplitude_change(&mut self, change: u8) -> &mut Self {
        assert!((0..=15).contains(&change), "Max amplitude change must be between 0 and 15.");
        self.pwm_config.max_amplitude_change = change;
        self
    }

    pub fn freewheel_mode(&mut self, mode: FreewheelMode) -> &mut Self {
        self.pwm_config.freewheel_mode = mode;
        self
    }

    pub fn pwm_autogradient(&mut self, enable: bool) -> &mut Self {
        self.pwm_config.autogradient = enable;
        self
    }

    pub fn pwn_autoscale(&mut self, enable: bool) -> &mut Self {
        self.pwm_config.autoscale = enable;
        self
    }

    pub fn pwm_frequency(&mut self, frequency: PwmFrequency) -> &mut Self {
        self.pwm_config.frequency = frequency;
        self
    }

    pub fn pwm_gradient(&mut self, gradient: u8) -> &mut Self {
        self.pwm_config.gradient = gradient;
        self
    }

    pub fn pwm_offset(&mut self, offset: u8) -> &mut Self {
        self.pwm_config.offset = offset;
        self
    }

    // ========================
    // ====Motion Control======
    // ========================
    pub fn step_loop(&mut self, target_rpm: f64, accel_rpm2: f64) -> Result<()> {
        // Calculate microstep multiplier based on resolution
        let msteps_per_step = match self.driver_config.microstep_resolution {
            MicrostepResolution::M256 => 256,
            MicrostepResolution::M128 => 128,
            MicrostepResolution::M64 => 64,
            MicrostepResolution::M32 => 32,
            MicrostepResolution::M16 => 16,
            MicrostepResolution::M8 => 8,
            MicrostepResolution::M4 => 4,
            MicrostepResolution::M2 => 2,
            MicrostepResolution::M1 => 1,
        } as f64;
        let pulses_per_mstep = if self.driver_config.double_edge_step {1.0} else {2.0};
        let target_v = target_rpm / 60.0;
        let a = accel_rpm2 / 60.0_f64.powi(2);
        let mut v = 0.0;
        let mut t = 0.01;
        let temp = 1.0 / (pulses_per_mstep * msteps_per_step * self.steps_per_rev * target_v);
        log::info!("V====={temp}\n");
        let mut i: u64 = 0;
        loop {
            let dt = 2.0 * t;
            v = (v + a * dt).min(target_v);
            t = 1.0 / (pulses_per_mstep * msteps_per_step * self.steps_per_rev * v);
            let delay = Duration::from_secs_f64(t);
            self.step_pin.set_high()?;
            std::thread::sleep(delay);
            self.step_pin.set_low()?;
            std::thread::sleep(delay);
            i += 1;
        }
    }

    // ========================
    // ====Status Functions====
    // ========================
    // Global Status
    pub fn charge_pump_undervoltage(&self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.charge_pump_undervoltage)
        } else {
            Err(Tmc2209Error::InvalidResponse("Global status has incorrect register".to_string()))
        }
    }

    pub fn driver_error(&self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.driver_error)
        } else {
            Err(Tmc2209Error::InvalidResponse("Global status has incorrect register".to_string()))
        }
    }

    pub fn is_reset(&self) -> Result<bool> {
        if let FrameData::GlobalStatus(data) = self.read_register(GLOBAL_STATUS_REG)? {
            Ok(data.reset)
        } else {
            Err(Tmc2209Error::InvalidResponse("Global status has incorrect register".to_string()))
        }
    }

    pub fn transmission_count(&self) -> Result<u8> {
        if let FrameData::TransmissionCount(x) = self.read_register(TRANSMISSION_COUNT_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse("Transmission count has incorrect register".to_string()))
        }
    }

    pub fn version(&self) -> Result<u8> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.version)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn direction_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.direction)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn disable_pwm_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.disable_pwm)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }
    pub fn step_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.step)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn powerdown_uart_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.powerdown_uart)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }


    pub fn diagnostic_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.diagnostic)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn microstep2_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.microstep2)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn microstep1_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.microstep1)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn disable_pin(&self) -> Result<bool> {
        if let FrameData::PinStates(data) = self.read_register(PIN_STATES_REG)? {
            Ok(data.disable)
        } else {
            Err(Tmc2209Error::InvalidResponse("Pin states has incorrect register".to_string()))
        }
    }

    pub fn microstep_time(&self) -> Result<u32> {
        if let FrameData::MicrostepTime(x) = self.read_register(MICROSTEP_TIME_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse("Microstep time has incorrect register".to_string()))
        }
    }

    pub fn motor_load(&self) -> Result<u16> {
        if let FrameData::MotorLoad(x) = self.read_register(MOTOR_LOAD_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse("Motor load has incorrect register".to_string()))
        }
    }

    pub fn microstep_position(&self) -> Result<u16> {
        if let FrameData::MicrostepPosition(x) = self.read_register(MICROSTEP_POSITION_REG)? {
            Ok(x)
        } else {
            Err(Tmc2209Error::InvalidResponse("Microstep position has incorrect register".to_string()))
        }
    }

    pub fn microstep_current(&self) -> Result<(i16, i16)> {
        if let FrameData::MicrostepCurrent(a, b) = self.read_register(MICROSTEP_CURRENT_REG)? {
            Ok((a, b))
        } else {
            Err(Tmc2209Error::InvalidResponse("Microstep current has incorrect register".to_string()))
        }
    }

    pub fn stopped(&self) -> Result<bool> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.stopped)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn pwm_mode(&self) -> Result<bool> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.pwm_mode)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn current_scale(&self) -> Result<u8> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.current_scale)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn temperature(&self) -> Result<TemperatureThreshold> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.temperature)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn open_load(&self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.open_load)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn low_side_short(&self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.low_side_short)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn ground_short(&self) -> Result<PhaseStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.ground_short)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn overtemperature(&self) -> Result<OvertemperatureStatus> {
        if let FrameData::DriverStatus(data) = self.read_register(DRIVER_STATUS_REG)? {
            Ok(data.overtemperature)
        } else {
            Err(Tmc2209Error::InvalidResponse("Driver status has incorrect register".to_string()))
        }
    }

    pub fn pwm_state(&self) -> Result<PwmState> {
        let mut state = PwmState {
            auto_grad: 0,
            auto_offset: 0,
            scale: 0,
            offset: 0,
        };
        let FrameData::PwmAuto(data) = self.read_register(PWM_AUTO_REG)? else {
            return Err(Tmc2209Error::InvalidResponse("PWM auto has incorrect register".to_string()));
        };
        state.auto_grad = data.gradient;
        state.auto_offset = data.offset;
        let FrameData::PwmScale(data) = self.read_register(PWM_SCALE_REG)? else {
            return Err(Tmc2209Error::InvalidResponse("PWM scale has incorrect register".to_string()));
        };
        state.scale = data.scale;
        state.offset = data.offset;
        Ok(state)
    }

    // ===========================
    // ==== Utility Functions ====
    // ===========================
    fn write_register(&mut self, register: u8, data: FrameData) -> Result<()> {
        // Read transmission count before write
        let count_before = self.transmission_count()?;

        let frame = WriteFrame {
            address: self.address,
            register,
            data,
        };

        // Write frame to buffer with extra byte for CRC.
        let mut buf = [0u8; 8];
        let mut cursor = Cursor::new(&mut buf[..7]);
        let mut writer = deku::writer::Writer::new(&mut cursor);
        frame.to_writer(&mut writer, ())?;

        // Calculate and append CRC
        buf[7] = crc8(&buf[0..7]);

        log::info!(
            "Writing register {:#04x}, frame: {:02x?}",
            register,
            buf
        );

        self.uart.write(&buf)?;
        std::thread::sleep(Duration::from_millis(5));

        // Read back the echo since TX and RX are on the same line
        let mut echo = [0u8; 8];
        self.uart.read(&mut echo, self.timeout.0)?;

        // Verify echo matches what we sent
        if echo != buf {
            return Err(Tmc2209Error::InvalidResponse(format!(
                "write echo mismatch: sent {:02x?}, received {:02x?}",
                buf, echo
            )));
        }

        // Verify transmission count incremented
        let count_after = self.transmission_count()?;
        if count_after != count_before.wrapping_add(1) {
            return Err(Tmc2209Error::InvalidResponse(format!(
                "transmission count mismatch",
            )));
        }

        std::thread::sleep(Duration::from_millis(10));
        Ok(())
    }

    pub fn read_register(&self, register: u8) -> Result<FrameData> {
        // Create read request frame
        let request = ReadRequestFrame {
            address: self.address,
            register,
        };

        // Write request to buffer with extra byte for CRC
        let mut request_buf = [0u8; 4];
        let mut cursor = Cursor::new(&mut request_buf[..3]);
        let mut writer = deku::writer::Writer::new(&mut cursor);
        request.to_writer(&mut writer, ())?;

        // Calculate and append CRC
        request_buf[3] = crc8(&request_buf[0..3]);

        log::debug!(
            "Reading register {:#04x}, request: {request_buf:02x?}",
            register
        );

        self.uart.write(&request_buf)?;
        std::thread::sleep(Duration::from_millis(10));

        // Read back the echo since TX and RX are on the same line
        let mut echo = [0u8; 4];
        self.uart.read(&mut echo, self.timeout.0)?;

        // Verify echo matches what we sent
        if echo != request_buf {
            return Err(Tmc2209Error::InvalidResponse(format!(
                "read request echo mismatch: sent {:02x?}, received {:02x?}",
                request_buf, echo
            )));
        }

        // Wait for response
        std::thread::sleep(Duration::from_millis(10));

        // Read response (8 bytes)
        let mut buf = [0u8; 8];
        let mut total_read = 0;
        while total_read < 8 {
            total_read += self.uart.read(&mut buf[total_read..], self.timeout.0)?;
        }

        log::debug!("Read response: {buf:02x?}");

        // Verify CRC before parsing
        let received_crc = buf[7];
        let calculated_crc = crc8(&buf[0..7]);
        if received_crc != calculated_crc {
            return Err(Tmc2209Error::CrcMismatch(format!(
                "register read: expected {:#04x}, got {:#04x}",
                calculated_crc, received_crc
            )));
        }

        // Parse response frame
        let (_rest, response) = ReadResponseFrame::from_bytes((&buf, 0))?;

        Ok(response.data)
    }

    fn get_current_scale(&mut self, rms_amps: f64) -> u8 {
        let ohms = self.sense_ohms + 0.02;
        let volts = if self.driver_config.low_sense_resistor_voltage {0.180} else {0.325};
        let scale = 2.0_f64.sqrt() * 32.0 * rms_amps * ohms / volts - 1.0;
        log::info!("{scale}\n");
        scale.round().clamp(0.0, 31.0) as u8
    }
}

fn crc8(data: &[u8]) -> u8 {
    // CRC-8 polynomial 0x07, init 0x00, no reflect, xorout 0x00
    // Matches TMC2209 datasheet
    let mut crc: u8 = 0x00;
    for &byte in data {
        let mut byte = byte;
        for _ in 0..8 {
            crc = if ((crc >> 7) ^ (byte & 0x01)) != 0 {
                (crc << 1) ^ 0x07
            } else {
                crc << 1
            };
            byte = byte >> 1;
        }
    }
    crc
}
