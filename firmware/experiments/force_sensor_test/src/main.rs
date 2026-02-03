#![no_std]
#![no_main]

mod fmt;
mod force_sensor;
mod protocol;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AnyAdcChannel, AdcChannel, Resolution, SampleTime};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::{Duration, Timer};
use fmt::info;
use protocol::{build_frame, MessageType};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut adc = Adc::new(p.ADC1, SampleTime::CYCLES247_5, Resolution::BITS12);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let mut uart = Uart::new_blocking(p.USART1, p.PA10, p.PA9, uart_config).unwrap();

    // Enable/disable sensors here. Disabled sensors transmit 0.
    const SENSOR_ENABLED: [bool; 10] = [true, true, true, true, true, true, true, true, true, true];

    // Update pins to match your wiring. These are 10 distinct ADC channels.
    let mut channels: [AnyAdcChannel<_>; 10] = [
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
        p.PA2.degrade_adc(),
        p.PA3.degrade_adc(),
        p.PA4.degrade_adc(),
        p.PA5.degrade_adc(),
        p.PA6.degrade_adc(),
        p.PA7.degrade_adc(),
        p.PB0.degrade_adc(),
        p.PB1.degrade_adc(),
    ];

    let mut readings: [u16; 10] = [0; 10];
    loop {
        for (idx, channel) in channels.iter_mut().enumerate() {
            readings[idx] = if SENSOR_ENABLED[idx] {
                adc.blocking_read(channel)
            } else {
                0
            };
        }

        let frame = build_frame(MessageType::ForceReadings as u8, readings);
        let _ = uart.blocking_write(&frame.buf[..frame.len]);
        info!("ADC array raw: {}", readings);
        Timer::after(Duration::from_millis(50)).await;
    }
}
