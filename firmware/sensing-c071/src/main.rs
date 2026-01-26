#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{adc::{Adc, AdcChannel, Resolution, SampleTime}, gpio::{Level, Output, Speed}};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);


    let mut adc = Adc::new(p.ADC1, Resolution::BITS12);
    let mut pin = p.PA0.degrade_adc();
    let fsr_force_min = 0.0_f32; // grams
    let fsr_force_max = 500.0_f32; // grams
    let fsr_bits_min = 0_u16; // 12 bit reading
    let fsr_bits_max = 4095_u16; // 12 bit reading

    loop{
        led.toggle();
        let x = adc.blocking_read(&mut pin, SampleTime::CYCLES24_5);
        let t = ((x.saturating_sub(fsr_bits_min)) as f32)
            / ((fsr_bits_max - fsr_bits_min) as f32); // normalized force (between 0 and 1)
        let force = fsr_force_min + t * (fsr_force_max - fsr_force_min);
        info!("{}", force);
        Timer::after_millis(50).await;
    }
    

}
