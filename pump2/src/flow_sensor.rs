use esp_idf_svc::hal::{
    delay::TickType,
    gpio::{InputPin, OutputPin},
    i2c::{config::Config, I2c, I2cDriver},
    peripheral::Peripheral,
    units::Hertz,
};
use std::fmt;
use std::time::Duration;

const SENSOR_ADDR: u8 = 0x08;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Command {
    StartWater,
    StartIpa,
    Stop,
    SoftReset,
}

impl Command {
    fn as_bytes(&self) -> [u8; 2] {
        match self {
            Command::StartWater => [0x36, 0x08],
            Command::StartIpa => [0x36, 0x15],
            Command::Stop => [0x3F, 0xF9],
            Command::SoftReset => [0x00, 0x06],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LiquidType {
    Water,
    Ipa,
}

#[derive(Debug)]
pub enum FlowSensorError {
    I2cError(String, esp_idf_svc::sys::EspError),
    CrcMismatch(String),
}

impl fmt::Display for FlowSensorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FlowSensorError::I2cError(context, e) => {
                write!(f, "I2C error during {}: {}", context, e)
            }
            FlowSensorError::CrcMismatch(context) => write!(f, "CRC mismatch in {}", context),
        }
    }
}

impl std::error::Error for FlowSensorError {}

pub type Result<T> = std::result::Result<T, FlowSensorError>;

#[derive(Debug, Clone, Copy)]
pub struct FlowReading {
    pub air_in_line: bool,
    pub high_flow: bool,
    pub exponential_smoothing_active: bool,
    pub ul_per_min: f64,
    pub degrees_c: f64,
}

pub struct FlowSensor<'a> {
    i2c: I2cDriver<'a>,
    timeout: TickType,
}

impl<'a> FlowSensor<'a> {
    pub fn new(
        i2c: impl Peripheral<P = impl I2c> + 'a,
        sda: impl Peripheral<P = impl InputPin + OutputPin> + 'a,
        scl: impl Peripheral<P = impl InputPin + OutputPin> + 'a,
    ) -> Result<Self> {
        let i2c = I2cDriver::new(i2c, sda, scl, &Config::new().baudrate(Hertz(400_000)))
            .map_err(|e| FlowSensorError::I2cError("initialization".to_string(), e))?;
        Ok(Self {
            i2c,
            timeout: TickType::from(Duration::from_secs(1)),
        })
    }

    pub fn timeout(&mut self, timeout: Duration) -> &mut Self {
        self.timeout = TickType::from(timeout);
        self
    }

    pub fn start(&mut self, liquid: LiquidType) -> Result<()> {
        self.stop()?;
        std::thread::sleep(Duration::from_millis(20));

        let cmd = match liquid {
            LiquidType::Water => Command::StartWater,
            LiquidType::Ipa => Command::StartIpa,
        };
        let cmd_bytes = cmd.as_bytes();
        log::info!("Starting flow sensor with command: {cmd_bytes:#02x?}");
        self.i2c
            .write(SENSOR_ADDR, &cmd_bytes, self.timeout.0)
            .map_err(|e| FlowSensorError::I2cError("start".to_string(), e))?;
        std::thread::sleep(Duration::from_millis(20));
        Ok(())
    }

    pub fn reset(&mut self) -> Result<()> {
        self.i2c
            .write(SENSOR_ADDR, &Command::SoftReset.as_bytes(), self.timeout.0)
            .map_err(|e| FlowSensorError::I2cError("reset".to_string(), e))?;
        Ok(())
    }

    pub fn read(&mut self) -> Result<FlowReading> {
        let mut buf = [0u8; 9];
        self.i2c
            .read(SENSOR_ADDR, &mut buf, self.timeout.0)
            .map_err(|e| FlowSensorError::I2cError("read".to_string(), e))?;

        // Verify CRCs
        if crc8([buf[0], buf[1]]) != buf[2] {
            return Err(FlowSensorError::CrcMismatch("flow rate data".to_string()));
        }
        if crc8([buf[3], buf[4]]) != buf[5] {
            return Err(FlowSensorError::CrcMismatch("temperature data".to_string()));
        }
        if crc8([buf[6], buf[7]]) != buf[8] {
            return Err(FlowSensorError::CrcMismatch("status data".to_string()));
        }

        // Parse flow rate (sensor reports in µL/min scaled by 10)
        let flow_raw = i16::from_be_bytes([buf[0], buf[1]]);
        let ul_per_min = flow_raw as f64 / 10.0;

        // Parse temperature
        let temp_raw = i16::from_be_bytes([buf[3], buf[4]]);
        let degrees_c = temp_raw as f64 / 200.0;

        // Parse status word
        let status = u16::from_be_bytes([buf[6], buf[7]]);
        let air_in_line = (status & 0x0001) != 0;
        let high_flow = (status & 0x0002) != 0;
        let exponential_smoothing_active = (status & 0x0004) != 0;

        Ok(FlowReading {
            air_in_line,
            high_flow,
            exponential_smoothing_active,
            ul_per_min,
            degrees_c,
        })
    }

    pub fn stop(&mut self) -> Result<()> {
        self.i2c
            .write(SENSOR_ADDR, &Command::Stop.as_bytes(), self.timeout.0)
            .map_err(|e| FlowSensorError::I2cError("stop".to_string(), e))?;
        Ok(())
    }
}

fn crc8(two_bytes: [u8; 2]) -> u8 {
    // CRC-8 polynomial 0x31, init 0xFF, no reflect, xorout 0x00
    // Matches datasheet Table 15.
    let mut crc: u8 = 0xFF;
    for &b in &two_bytes {
        crc ^= b;
        for _ in 0..8 {
            crc = if (crc & 0x80) != 0 {
                (crc << 1) ^ 0x31
            } else {
                crc << 1
            };
        }
    }
    crc
}
