use banana_hand_common::{FORCE_DATA_PACKET_LEN, FORCE_SENSOR_COUNT, ForceDataPacket};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::config::FORCE_COUNT;
use crate::shared::SharedData;

const _: [(); FORCE_COUNT] = [(); FORCE_SENSOR_COUNT];
const _: [(); FORCE_SENSOR_COUNT] = [(); FORCE_COUNT];

#[embassy_executor::task]
pub async fn c0_reader_task(
    mut rx: UartRx<'static, Async>,
    shared: &'static SharedData<FORCE_COUNT>,
) {
    let mut readings = [0u16; FORCE_COUNT];
    let mut rx_buf = [0u8; 64];
    let mut packet_buf = [0u8; FORCE_DATA_PACKET_LEN];
    let mut packet_len = 0usize;

    loop {
        let n = match rx.read_until_idle(&mut rx_buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };

        for &b in &rx_buf[..n] {
            packet_buf[packet_len] = b;
            packet_len += 1;

            if packet_len != FORCE_DATA_PACKET_LEN {
                continue;
            }

            match ForceDataPacket::parse(&packet_buf) {
                Ok(packet) => {
                    readings.copy_from_slice(&packet.into_readings());
                    shared.write_frame(&readings);
                    packet_len = 0;
                }
                Err(_) => {
                    // No frame delimiter is used on this link, so resync by sliding one byte.
                    packet_buf.copy_within(1..FORCE_DATA_PACKET_LEN, 0);
                    packet_len = FORCE_DATA_PACKET_LEN - 1;
                }
            }
        }
    }
}
