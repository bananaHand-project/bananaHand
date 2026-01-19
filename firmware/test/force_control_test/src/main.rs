#![no_std]
#![no_main]

mod fmt;
mod force_sensor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    adc::{Adc, AdcChannel, Resolution, SampleTime},
    gpio::{Level, Output, Speed},
};
use embassy_time::{Duration, Timer};
use fmt::info;
use force_sensor::ForceSensor;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PB7, Level::High, Speed::Low);
    let mut adc = Adc::new(p.ADC1, Resolution::BITS12);

    let mut force_sensor = ForceSensor::new(p.PA0.degrade_adc(), SampleTime::CYCLES247_5);
    loop {
        let force = force_sensor.read_force_blocking(&mut adc);
        info!("Force sensor force: {} g", force);
        Timer::after(Duration::from_millis(50)).await;
    }
}