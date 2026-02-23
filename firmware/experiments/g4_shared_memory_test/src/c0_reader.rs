use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::protocol::{FrameParser, MessageType};
use crate::shared_data::SharedData;
use defmt::info;

#[embassy_executor::task]
pub async fn c0_reader_task(
    mut rx: UartRx<'static, Async>,
    shared: &'static SharedData<10>,
) {
    let mut parser = FrameParser::new();
    let mut readings = [0u16; 10];
    let mut frame_count: u32 = 0;
    let mut rx_buf = [0u8; 64];

    loop {
        let n = match rx.read_until_idle(&mut rx_buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };

        for &b in &rx_buf[..n] {
            if let Some((msg_type, payload)) = parser.parse_byte(b) {
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
}
