use alloc::{
    vec,
    vec::Vec,
};
use core::{
    result,
    slice,
};
use defmt::info;
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
}

impl From<TimeoutError> for Error {
    fn from(_: TimeoutError) -> Self { Error::Timeout }
}

type Result<T> = result::Result<T, Error>;

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(endian = "big")]
struct RequestPacket {
    type: u8,
    #[deku(ctx = "*type")]
    data: RequestPacketData,
    crc: u8,
}

#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian, register: u8", id = "type", endian = "endian")]
enum RequestPacketData {
    #[deku(id = "SENSOR_INFO_PACKET")]
    SensorInfo,

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
}

// General configuration registers.
#[derive(Debug, Clone, Copy, DekuRead, DekuWrite)]
#[deku(ctx = "endian: deku::ctx::Endian", endian = "endian")]
struct SensorInfo {
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

pub async fn protocol_task(device: USB_DEVICE<'_>) -> ! {
    let mut stream = PacketStream::new(device, Duration::from_secs(1));
    loop {
        let packet = stream.read().await.unwrap();
        info!("Received packet: {=[?]}", &packet[..]);
        stream.write(&packet).await;
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


        Write::write_all(&mut self.usb, &out).await.unwrap()
    }

    pub async fn read(&mut self) -> Result<Vec<u8>> {
        let mut size = 0;
        while size == 0 {
            size = self.read_byte().await?;
            info!("Read zero byte");
        }

        let mut vec = Vec::new();
        loop {
            let l = vec.len();
            vec.resize(l + size as usize - 1, 0);
            info!("Reading {} bytes", size);
            embassy_time::with_timeout(
                self.timeout,
                Read::read_exact(&mut self.usb, &mut vec[l..]),
            ).await?.unwrap();
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
        embassy_time::with_timeout(
            self.timeout,
            Read::read_exact(&mut self.usb, slice::from_mut(&mut byte)),
        ).await?.unwrap();
        Ok(byte)
    }
}
