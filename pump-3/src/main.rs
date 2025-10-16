#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

mod usb;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_hal::{
    self,
    clock::CpuClock,
    timer::timg::TimerGroup,
};
use panic_rtt_target as _;
use crate::usb::Usb;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let peripherals = esp_hal::init(
        esp_hal::Config::default().with_cpu_clock(CpuClock::max()),
    );

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);
    esp_rtos::start(TimerGroup::new(peripherals.TIMG0).timer0);
    info!("Embassy initialized!");

    spawner.spawn(server()).unwrap();
    Timer::after_secs(4).await;

    let mut usb = Usb::new(peripherals.USB_DEVICE);
    loop {
        info!("Sending!");
        usb.write(&[0]).await;
        info!("Reading!");
        let res = usb.read().await;
        info!("Got {=[?]}", &res[..]);
        Timer::after_secs(2).await;
        // info!("{}", res);
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
