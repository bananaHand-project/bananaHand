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
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use fmt::info;
use protocol::{build_frame, MessageType};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut adc = Adc::new(p.ADC1, SampleTime::CYCLES247_5, Resolution::BITS12);

    // C0 -> G4 force stream.
    let mut g4_uart_config = UartConfig::default();
    g4_uart_config.baudrate = 115_200;
    let mut g4_uart = UartTx::new_blocking(p.USART1, p.PB6, g4_uart_config).unwrap();

    // Enable/disable sensors here. Disabled sensors transmit 0.
    const SENSOR_ENABLED: [bool; 10] = [true, true, true, true, true, true, true, true, true, true];

    // C071RB board-aware ADC pinout:
    // avoid PA2/PA3 (USART2 VCP default on MB2046) and PA5 (user LED).
    let mut channels: [AnyAdcChannel<_>; 10] = [
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
        p.PA4.degrade_adc(),
        p.PA6.degrade_adc(),
        p.PA7.degrade_adc(),
        p.PA8.degrade_adc(),
        p.PB0.degrade_adc(),
        p.PB1.degrade_adc(),
        p.PB10.degrade_adc(),
        p.PB11.degrade_adc(),
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
        let _ = g4_uart.blocking_write(&frame.buf[..frame.len]);
        info!("ADC array raw: {}", readings);
    }
}
