#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::{
    Config,
    clock::CpuClock,
    usb_serial_jtag::UsbSerialJtag,
};
use esp_hal::{
    timer::systimer::SystemTimer,
};
use panic_rtt_target as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(
        Config::default().with_cpu_clock(CpuClock::max()),
    );

    esp_hal_embassy::init(SystemTimer::new(peripherals.SYSTIMER).alarm0);
    spawner.spawn(server()).unwrap();

    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    loop {
        info!("Hello world!");
        let _ = usb.write(b"USB-Serial/JTAG echo ready\r\n");
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn server() -> ! {
    loop {
        info!("Hello from second task!");
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn task() -> ! {
    loop {
        info!("Hello from second task!");
        Timer::after_millis(500).await;
    }
}
