use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;
use embassy_time::Timer;

use crate::config::{FORCE_COUNT, POSITION_COUNT};
use crate::protocol::build_telemetry_frame;
use crate::shared::SharedData;

#[task]
pub async fn telemetry_sender_task(
    mut tx: UartTx<'static, Async>,
    shared_positions: &'static SharedData<POSITION_COUNT>,
    shared_forces: &'static SharedData<FORCE_COUNT>,
) {
    loop {
        let positions = shared_positions.read_snapshot();
        let forces = shared_forces.read_snapshot();
        let frame = build_telemetry_frame(positions, forces);
        let _ = tx.write(&frame.buf[..frame.len]).await;
        // Fixed telemetry rate to PC.
        Timer::after_millis(50).await;
    }
}
