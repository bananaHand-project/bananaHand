use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::protocol::{FrameParser, MessageType};
use crate::shared::COMMANDS;

#[task]
pub async fn command_reader_task(mut rx: UartRx<'static, Async>) {
    let mut rx_buf = [0u8; 64];
    let mut parser = FrameParser::new();

    loop {
        let n = rx.read_until_idle(&mut rx_buf).await.unwrap();

        for &b in &rx_buf[..n] {
            if let Some((msg_type, payload)) = parser.parse_byte(b) {
                if msg_type == MessageType::PositionUpdate as u8 {
                    if payload.len() != 16 {
                        defmt::warn!("PositionUpdate payload wrong size: {}", payload.len());
                        continue;
                    }

                    let mut commands = [0u16; 8];
                    for i in 0..8 {
                        let start = i * 2;
                        let bytes: [u8; 2] = payload[start..start + 2].try_into().unwrap();
                        commands[i] = u16::from_le_bytes(bytes);
                    }

                    COMMANDS.store(commands);
                } else {
                    defmt::info!("Unhandled message type {}", msg_type);
                }
            }
        }
    }
}
