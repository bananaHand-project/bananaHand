use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;
use embassy_time::Timer;

use crate::protocol::{build_frame, MessageType};
use crate::shared::POSITIONS;

#[task]
pub async fn telemetry_sender_task(mut tx: UartTx<'static, Async>) {
    loop {
        let positions = POSITIONS.read_snapshot();
        let frame = build_frame(MessageType::PositionUpdate as u8, positions);
        let _ = tx.write(&frame.buf[..frame.len]).await;
        Timer::after_millis(100).await;
    }
}
