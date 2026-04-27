#![no_std]
#![no_main]

mod fmt;
// mod force_sensor;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use banana_hand_common::{
    FORCE_DATA_PACKET_LEN, FORCE_SENS_BAUD, FORCE_SENSOR_COUNT, FORCE_SENSOR_INDEX_IDX,
    FORCE_SENSOR_MIDDLE_IDX, FORCE_SENSOR_PALM_1_IDX, FORCE_SENSOR_PALM_2_IDX, FORCE_SENSOR_PALM_3_IDX,
    FORCE_SENSOR_PALM_4_IDX, FORCE_SENSOR_PALM_5_IDX, FORCE_SENSOR_PINKY_IDX, FORCE_SENSOR_RING_IDX,
    FORCE_SENSOR_THUMB_IDX, encode_readings_unchecked,
};
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, AnyAdcChannel, Resolution, SampleTime};
use embassy_stm32::usart::{Config as UartConfig, UartTx};
use embassy_time::{Duration, Ticker};
#[cfg(feature = "debug")]
use fmt::info;

const SAMPLE_PERIOD: Duration = Duration::from_millis(5); // 200 Hz
#[cfg(feature = "debug")]
const FORCE_LOG_EVERY_SAMPLES: u32 = 20; // 10 Hz at 200 Hz sample rate

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

    // Fill channel slots by canonical packet indices from banana_hand_common.
    let mut channels_by_slot: [Option<AnyAdcChannel<_>>; FORCE_SENSOR_COUNT] =
        [const { None }; FORCE_SENSOR_COUNT];
    channels_by_slot[FORCE_SENSOR_INDEX_IDX] = Some(p.PA0.degrade_adc());
    channels_by_slot[FORCE_SENSOR_MIDDLE_IDX] = Some(p.PA1.degrade_adc());
    channels_by_slot[FORCE_SENSOR_RING_IDX] = Some(p.PA2.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PINKY_IDX] = Some(p.PA3.degrade_adc());
    channels_by_slot[FORCE_SENSOR_THUMB_IDX] = Some(p.PA8.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PALM_1_IDX] = Some(p.PA4.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PALM_2_IDX] = Some(p.PA5.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PALM_3_IDX] = Some(p.PA6.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PALM_4_IDX] = Some(p.PA7.degrade_adc());
    channels_by_slot[FORCE_SENSOR_PALM_5_IDX] = Some(p.PB2.degrade_adc());
    let mut channels: [AnyAdcChannel<_>; FORCE_SENSOR_COUNT] =
        channels_by_slot.map(|slot| slot.unwrap());

    let mut readings: [u16; FORCE_SENSOR_COUNT] = [0; FORCE_SENSOR_COUNT];
    let mut encoded: [u8; FORCE_DATA_PACKET_LEN] = [0; FORCE_DATA_PACKET_LEN];
    let mut ticker = Ticker::every(SAMPLE_PERIOD);
    #[cfg(feature = "debug")]
    let mut sample_counter: u32 = 0;
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
        #[cfg(feature = "debug")]
        {
            sample_counter = sample_counter.wrapping_add(1);
            if sample_counter % FORCE_LOG_EVERY_SAMPLES == 0 {
                info!("force: {}", readings);
            }
        }
    }
}
