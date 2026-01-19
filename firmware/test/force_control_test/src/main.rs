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
};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::{Duration, Timer};
use heapless::String;
use core::fmt::Write;
use fmt::info;
use force_sensor::ForceSensor;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut adc = Adc::new(p.ADC1, SampleTime::CYCLES247_5, Resolution::BITS12);
    let mut force_sensor = ForceSensor::new(p.PA0.degrade_adc());

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let mut uart = Uart::new_blocking(p.USART1, p.PA10, p.PA9, uart_config).unwrap();

    loop {
        let force = force_sensor.read_force_blocking(&mut adc);
        let force_tenths = (force * 10.0 + 0.5) as u32;
        let force_whole = force_tenths / 10;
        let force_frac = force_tenths % 10;

        let mut line: String<64> = String::new();
        let _ = write!(line, "force_g={}.{}\r\n", force_whole, force_frac);
        let _ = uart.blocking_write(line.as_bytes());

        info!("Force sensor force: {} g", force);
        Timer::after(Duration::from_millis(50)).await;
    }
}
