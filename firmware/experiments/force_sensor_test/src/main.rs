#![no_std]
#![no_main]

mod fmt;
// mod force_sensor;
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

// Keep these indices aligned with uart_g4_combo control_config FORCE_MAPS:
// 0 ring, 1 pinky, 2 thumb, 3 index_1, 4 middle, 5 index_2.
const FORCE_RING: usize = 0;
const FORCE_PINKY: usize = 1;
const FORCE_THUMB: usize = 2;
const FORCE_INDEX_1: usize = 3;
const FORCE_MIDDLE: usize = 4;
const FORCE_INDEX_2: usize = 5;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut adc = Adc::new(p.ADC1, Resolution::BITS12);

    // C0 -> G4 force stream.
    let mut g4_uart_config = UartConfig::default();
    g4_uart_config.baudrate = 115_200;
    let mut g4_uart = UartTx::new_blocking(p.USART1, p.PC14, g4_uart_config).unwrap();

    // Enable/disable sensors here. Disabled sensors transmit 0.
    const SENSOR_ENABLED: [bool; 10] = [true, true, true, true, true, true, true, true, true, true];

    // Channel order is the exact order placed into the outgoing force frame.
    // Finger sensors are mapped to uart_g4_combo force indices 0..5.
    // Extra slots are filled with palm sensors.
    let mut channels: [AnyAdcChannel<_>; 10] = [
        p.PA2.degrade_adc(), // [0] ring
        p.PA3.degrade_adc(), // [1] pinky
        p.PA8.degrade_adc(), // [2] thumb
        p.PA0.degrade_adc(), // [3] index_1
        p.PA1.degrade_adc(), // [4] middle
        p.PA4.degrade_adc(), // [5] index_2 slot filled by palm
        p.PA6.degrade_adc(), // [6] palm
        p.PA7.degrade_adc(), // [7] palm
        p.PB0.degrade_adc(), // [8] palm
        p.PB1.degrade_adc(), // [9] palm
    ];

    let mut readings: [u16; 10] = [0; 10];
    loop {
        for (idx, channel) in channels.iter_mut().enumerate() {
            readings[idx] = if SENSOR_ENABLED[idx] {
                adc.blocking_read(channel, SampleTime::CYCLES24_5)
            } else {
                0
            };
        }

        // Simulated force readings
        // readings = [
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000,
        //     4000
        // ];


        let frame = build_frame(MessageType::ForceReadings as u8, readings);
        let _ = g4_uart.blocking_write(&frame.buf[..frame.len]);
        info!(
            "[ring,pink,thumb,idx1,mid,idx2,...]: [{},{},{},{},{},{}|... ]",
            readings[FORCE_RING],
            readings[FORCE_PINKY],
            readings[FORCE_THUMB],
            readings[FORCE_INDEX_1],
            readings[FORCE_MIDDLE],
            readings[FORCE_INDEX_2]
        );
    }
}
