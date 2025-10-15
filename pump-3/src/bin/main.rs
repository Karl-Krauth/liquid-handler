#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::Timer;
use embassy_usb::{
    Builder,
    driver::EndpointError,
    class::cdc_acm::{
        CdcAcmClass,
        State,
    },
};
use esp_hal::{
    Config,
    clock::CpuClock,
    otg_fs::{
        asynch,
        Usb,
    },
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

    /*
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut ep_out_buf = [0u8; 1024];
    let mut state = State::new();
    let mut builder = Builder::new(
        asynch::Driver::new(
            Usb::new(
                peripherals.USB0,
                peripherals.GPIO20,
                peripherals.GPIO19,
            ),
            &mut ep_out_buf,
            asynch::Config::default()
        ),
        embassy_usb::Config::new(0x303A, 0x3001),
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    loop {info!("Waiting for USB connection..."); Timer::after_secs(1).await;}
    let mut usb = builder.build();
    let usb_future = usb.run();

    let echo_future = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_future, echo_future).await;
    */
    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);
    loop {
        info!("Hello world!");
        let _ = usb.write(b"USB-Serial/JTAG echo ready\r\n");
        Timer::after_secs(1).await;
    }
    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}


async fn echo<'d>(class: &mut CdcAcmClass<'d, asynch::Driver<'d>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        // Echo back in upper case
        for c in buf[0..n].iter_mut() {
            if 0x61 <= *c && *c <= 0x7a {
                *c &= !0x20;
            }
        }
        let data = &buf[..n];
        class.write_packet(data).await?;
    }
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
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
