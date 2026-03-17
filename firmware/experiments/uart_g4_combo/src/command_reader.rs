use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartRx;

use crate::config::COMMAND_COUNT;
use crate::control_config::ControlMode;
use crate::protocol::{FrameParser, MessageType};
use crate::shared::{SharedData, SharedMode};

fn parse_u16_payload(payload: &[u8], out: &mut [u16; COMMAND_COUNT]) -> bool {
    if payload.len() != COMMAND_COUNT * 2 {
        return false;
    }

    for (idx, slot) in out.iter_mut().enumerate() {
        let start = idx * 2;
        let bytes: [u8; 2] = payload[start..start + 2].try_into().unwrap();
        *slot = u16::from_le_bytes(bytes);
    }

    true
}

fn parse_control_mode_payload(payload: &[u8]) -> Option<ControlMode> {
    let mut values = [0u16; COMMAND_COUNT];
    if !parse_u16_payload(payload, &mut values) {
        return None;
    }

    ControlMode::from_wire(values[0] as u8)
}

#[task]
pub async fn command_reader_task(
    mut rx: UartRx<'static, Async>,
    shared_position_commands: &'static SharedData<COMMAND_COUNT>,
    shared_force_commands: &'static SharedData<COMMAND_COUNT>,
    shared_mode: &'static SharedMode,
) {
    let mut rx_buf = [0u8; 64];
    let mut parser = FrameParser::new();
    let mut commands = [0u16; COMMAND_COUNT];

    loop {
        let n = match rx.read_until_idle(&mut rx_buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };

        for &b in &rx_buf[..n] {
            if let Some((msg_type, payload)) = parser.parse_byte(b) {
                match msg_type {
                    x if x == MessageType::PositionUpdate as u8 => {
                        if !parse_u16_payload(payload, &mut commands) {
                            defmt::warn!("PositionUpdate payload wrong size: {}", payload.len());
                            continue;
                        }
                        shared_position_commands.write_frame(&commands);
                    }
                    x if x == MessageType::ForceCommand as u8 => {
                        if !parse_u16_payload(payload, &mut commands) {
                            defmt::warn!("ForceCommand payload wrong size: {}", payload.len());
                            continue;
                        }
                        shared_force_commands.write_frame(&commands);
                    }
                    x if x == MessageType::ControlModeUpdate as u8 => {
                        let Some(mode) = parse_control_mode_payload(payload) else {
                            defmt::warn!(
                                "Invalid ControlModeUpdate payload size/value: {}",
                                payload.len()
                            );
                            continue;
                        };

                        shared_mode.write(mode.to_wire());
                    }
                    _ => {
                        defmt::info!("Unhandled message type {}", msg_type);
                    }
                }
            }
        }
    }
}
