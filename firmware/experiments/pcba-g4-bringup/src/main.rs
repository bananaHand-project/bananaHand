#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PC14, Level::High, Speed::Low);

    loop {
        info!("Hello, World!");
        led1.set_high();
        led2.set_low();
        Timer::after(Duration::from_millis(500)).await;
        led1.set_low();
        led2.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}
