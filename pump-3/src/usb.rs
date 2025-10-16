use alloc::{
    string::String,
    vec,
    vec::Vec,
};
use core::{
    iter,
    slice,
};
use defmt::info;
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


#[derive(Debug, Error)]
pub enum Error {
    #[error("IO error")]
    IOError(String),
}

pub struct Usb<'a> {
    usb: UsbSerialJtag<'a, Async>,
}

impl<'a> Usb<'a> {
    pub fn new(device: USB_DEVICE<'a>) -> Self {
        let mut usb = UsbSerialJtag::new(device);
        Usb {
            usb: usb.into_async(),
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

    pub async fn read(&mut self) -> Vec<u8> {
        let mut size = 0;
        while size == 0 {
            size = self.read_byte().await;
            info!("Read zero byte");
        }

        let mut vec = Vec::new();
        loop {
            let l = vec.len();
            vec.resize(l + size as usize - 1, 0);
            info!("Reading {} bytes", size);
            Read::read_exact(&mut self.usb, &mut vec[l..]).await.unwrap();
            size = self.read_byte().await;
            match size {
                0 => break vec,
                255 => continue,
                _ => vec.push(0),
            }
        }
    }

    async fn read_byte(&mut self) -> u8 {
        let mut byte = 0u8;
        Read::read_exact(&mut self.usb, slice::from_mut(&mut byte)).await.unwrap();
        byte
    }

    async fn write_byte(&mut self, byte: u8) {
        Write::write_all(&mut self.usb, &[byte]).await.unwrap();
    }
}
