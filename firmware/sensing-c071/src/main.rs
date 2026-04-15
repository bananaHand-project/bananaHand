#![no_std]
#![no_main]

mod fmt;
// mod force_sensor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use banana_hand_common::{
    FORCE_DATA_PACKET_LEN, FORCE_SENS_BAUD, FORCE_SENSOR_COUNT, encode_readings_unchecked,
};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, Resolution, SampleTime};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Duration, Ticker};
use fmt::info;

// Keep these indices aligned with uart_g4_combo control_config FORCE_MAPS:
// 0 ring, 1 pinky, 2 thumb, 3 index_1, 4 middle, 5 index_2.
// const FORCE_INDEX: usize = 0;
// const FORCE_MIDDLE: usize = 1;
// const FORCE_RING: usize = 2;
// const FORCE_PINKY: usize = 3;
// const FORCE_THUMB: usize = 4;
// const FORCE_PALM_1: usize = 5;
// const FORCE_PALM_2: usize = 6;
// const FORCE_PALM_3: usize = 7;
// const FORCE_PALM_4: usize = 8;
// const FORCE_PALM_5: usize = 9;

const SAMPLE_PERIOD: Duration = Duration::from_millis(5); // 200 Hz

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut adc = Adc::new(p.ADC1, Resolution::BITS12);

    // C0 -> G4 force stream.
    let mut g4_uart_config = UartConfig::default();
    g4_uart_config.baudrate = FORCE_SENS_BAUD;
    let mut g4_uart = UartTx::new_blocking(p.USART1, p.PC14, g4_uart_config).unwrap();

    // Enable/disable sensors here. Disabled sensors transmit 0.
    const SENSOR_ENABLED: [bool; FORCE_SENSOR_COUNT] =
        [true, true, true, true, true, true, true, true, true, true];

    // Channel order is the exact order placed into the outgoing force frame.
    // Finger sensors are mapped to uart_g4_combo force indices 0..5.
    // Extra slots are filled with palm sensors.
    let mut channels: [AnyAdcChannel<_>; FORCE_SENSOR_COUNT] = [
        p.PA0.degrade_adc(), // [0] index
        p.PA1.degrade_adc(), // [1] middle
        p.PA2.degrade_adc(), // [2] ring
        p.PA3.degrade_adc(), // [3] pinky
        p.PA8.degrade_adc(), // [4] thumb
        p.PA4.degrade_adc(), // [5] palm 1
        p.PA5.degrade_adc(), // [6] palm 2
        p.PA6.degrade_adc(), // [7] palm 3
        p.PA7.degrade_adc(), // [8] palm 4
        p.PB2.degrade_adc(), // [9] palm 5
    ];

    let mut readings: [u16; FORCE_SENSOR_COUNT] = [0; FORCE_SENSOR_COUNT];
    let mut encoded: [u8; FORCE_DATA_PACKET_LEN] = [0; FORCE_DATA_PACKET_LEN];
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    loop {
        ticker.next().await;

        for (idx, channel) in channels.iter_mut().enumerate() {
            readings[idx] = if SENSOR_ENABLED[idx] {
                adc.blocking_read(channel, SampleTime::CYCLES24_5)
            } else {
                0
            };
        }

        // Simulated force readings for debug
        // readings = [4000; FORCE_SENSOR_COUNT];

        encode_readings_unchecked(&readings, &mut encoded);

        // In embassy-stm32 v0.5.0 this TX path is effectively infallible on Result
        // (blocking_write always return Ok(()))
        let _ = g4_uart.blocking_write(&encoded);
        info!("force: {}", readings);
    }
}
