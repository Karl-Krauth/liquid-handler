#[macro_use]
mod flow_sensor;
mod tmc2209;

use anyhow::Result;
use esp_idf_hal::{
    cpu::Core,
    delay::Delay,
    peripherals::Peripherals,
    task::thread::ThreadSpawnConfiguration,
};
use flow_sensor::{
    FlowSensor,
    LiquidType,
};
use std::thread;
use tmc2209::Tmc2209;

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    let delay = Delay::new_default();

    log::info!("Hello, world!");
    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;
    let sda = pins.gpio21;
    let scl = pins.gpio22;

    log::info!("Initializing TMC2209...");
    let tx = pins.gpio17;
    let rx = pins.gpio16;
    let step = pins.gpio18;
    let dir = pins.gpio19;
    let enable = pins.gpio23;

    ThreadSpawnConfiguration {
        name: Some(b"core1\0"),
        stack_size: 8 * 1024,
        priority: 20,
        inherit: false,
        pin_to_core: Some(Core::Core1),
        ..Default::default()
    }.set()?;
    let _thread = thread::spawn(move || -> Result<()> {
        log::info!("Initializing flow sensor...");
        let mut sensor = FlowSensor::new(peripherals.i2c0, sda, scl)?;
        log::info!("Flow sensor initialized successfully");

        log::info!("Starting sensor with water mode...");
        sensor.start(LiquidType::Water)?;
        log::info!("Sensor started successfully");
        loop {
            let reading = sensor.read()?;
            log::info!(
                "{}",
                reading.ul_per_min,
            );
            delay.delay_ms(500);
        }
    });

    let mut tmc = Tmc2209::new(
        peripherals.uart1,
        tx,
        rx,
        step,
        dir,
        enable,
    )?;

    // Configure TMC2209
    tmc.running_amps(0.707 * 0.8);

    log::info!("Reading TMC2209 status...");
    tmc.init()?;
    log::info!("TMC2209 initialized successfully");

    log::info!("Reading TMC2209 status...");
    // match tmc.status();
    log::info!("Enabling motor...");
    tmc.enable()?;

    log::info!("Starting motor at 10 RPM...");
    tmc.step_loop(500.0, 800.0)?;

    loop{};
}
