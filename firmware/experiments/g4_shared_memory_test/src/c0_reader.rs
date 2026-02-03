use embassy_stm32::mode::Blocking;
use embassy_stm32::usart::UartRx;

use crate::protocol::{FrameParser, MessageType};
use crate::shared_force::SharedForceData;
use defmt::info;

#[embassy_executor::task]
pub async fn c0_reader_task(
    mut rx: UartRx<'static, Blocking>,
    shared: &'static SharedForceData,
) {
    let mut parser = FrameParser::new();
    let mut readings = [0u16; 10];
    let mut frame_count: u32 = 0;

    loop {
        let mut byte = [0u8; 1];
        if rx.blocking_read(&mut byte).is_err() {
            continue;
        }
        if let Some((msg_type, payload)) = parser.parse_byte(byte[0]) {
            if msg_type != MessageType::ForceReadings as u8 {
                continue;
            }
            for idx in 0..10 {
                let lo = payload[idx * 2];
                let hi = payload[idx * 2 + 1];
                readings[idx] = u16::from_le_bytes([lo, hi]);
            }
            shared.write_frame(&readings);
            frame_count = frame_count.wrapping_add(1);
            if frame_count % 1 == 0 {
                info!(
                    "COBS frame {}: ch0={} ch1={} ch2={} ch3={}",
                    frame_count,
                    readings[0],
                    readings[1],
                    readings[2],
                    readings[3]
                );
            }
        }
    }
}
