use alloc::{
    vec,
    vec::Vec,
};
use core::{
    result,
    slice,
};
use defmt::{
    Format,
    info,
};
use deku::{
    ctx::BitSize,
    prelude::*,
};
use embassy_time::{
    Duration,
    TimeoutError,
};
use embedded_io_async::{
    Read,
    Write,
};
use esp_hal::{
    Async,
    peripherals::USB_DEVICE,
    usb_serial_jtag::UsbSerialJtag,
};
use thiserror::Error;


#[derive(Error, Debug)]
pub enum Error {
    #[error("Timeout error")]
    Timeout,
    #[error("Received unexpected zero byte")]
    ZeroByte,
}

impl From<TimeoutError> for Error {
    fn from(_: TimeoutError) -> Self { Error::Timeout }
}

type Result<T> = result::Result<T, Error>;

// Request/response codes.
const FLOW_SENSOR_INFO: u8 = 0x00;
const SET_PUMP_RPM: u8 = 0x01;
const FAIL: u8 = 0xFF;


#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Request {
    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo,

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm(f64),
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(id_type = "u8", endian = "big")]
enum Response {
    #[deku(id = "FLOW_SENSOR_INFO")]
    FlowSensorInfo(FlowSensorInfo),

    #[deku(id = "SET_PUMP_RPM")]
    SetPumpRpm,

    #[deku(id = "FAIL")]
    Fail,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite, Format)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct FlowSensorInfo {
    air_in_line: bool,
    high_flow: bool,
    exponential_smoothing_active: bool,
    ul_per_min: f64,
    degrees_c: f64,
}

pub async fn protocol_task(device: USB_DEVICE<'_>) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let Ok(packet) = stream.read().await else {
            continue;
        };
        info!("Received packet: {=[?]}", &packet[..]);
        let response = if let Ok((_, packet)) = Request::from_bytes((&packet, 0)) {
           match packet {
                Request::FlowSensorInfo => Response::FlowSensorInfo(FlowSensorInfo {
                    air_in_line: false,
                    high_flow: true,
                    exponential_smoothing_active: false,
                    ul_per_min: 100.32,
                    degrees_c: 33.1,
                }),
                Request::SetPumpRpm(rpm) => Response::SetPumpRpm,
            }
        } else {
            Response::Fail
        };

        info!("Sending response: {=[?]}", &response.to_bytes().unwrap()[..]);
        stream.write(&response.to_bytes().unwrap()).await;
    }
}

pub struct PacketStream<'a> {
    usb: UsbSerialJtag<'a, Async>,
    timeout: Duration,
}

impl<'a> PacketStream<'a> {
    pub fn new(device: USB_DEVICE<'a>, timeout: Duration) -> Self {
        PacketStream {
            usb: UsbSerialJtag::new(device).into_async(),
            timeout,
        }
    }

    pub async fn write(&mut self, data: &[u8]) {
        if data.len() == 0 {
            return;
        }

        let mut offset = 1;
        let mut offset_idx = 0;
        let mut out = vec![0];
        for &b in data {
            if b != 0 {
                out.push(b);
                offset += 1;
            }

            if b == 0 || offset == 255 {
                out.push(0);
                out[offset_idx] = offset;
                offset = 1;
                offset_idx = out.len() - 1;
            }
        }

        if (offset_idx != out.len() - 1) || (*data.last().unwrap() == 0) {
            out[offset_idx] = offset;
            out.push(0);
        }


        info!("Sending response: {=[?]}", &out[..]);
        Write::write_all(&mut self.usb, &out).await.unwrap()
    }

    pub async fn read(&mut self) -> Result<Vec<u8>> {
        let mut size = self.read_byte().await.unwrap();
        let mut vec = Vec::new();
        if size == 0 {
            return Ok(vec);
        }

        loop {
            let l = vec.len();
            vec.resize(l + size as usize - 1, 0);
            info!("Reading {} bytes", size);
            embassy_time::with_timeout(
                self.timeout,
                Read::read_exact(&mut self.usb, &mut vec[l..]),
            ).await?.unwrap();
            // Check for unexpected zero bytes.
            if vec[l..].iter().any(|&b| b == 0) {
                let mut byte = 1;
                while byte != 0 {
                    byte = self.read_byte().await.unwrap();
                }
                break Err(Error::ZeroByte);
            }

            size = self.read_byte().await?;
            match size {
                0 => break Ok(vec),
                255 => continue,
                _ => vec.push(0),
            }
        }
    }

    async fn read_byte(&mut self) -> Result<u8> {
        let mut byte = 0u8;
        Read::read_exact(&mut self.usb, slice::from_mut(&mut byte)).await.unwrap();
        Ok(byte)
    }
}
