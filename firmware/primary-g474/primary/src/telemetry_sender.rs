use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;
use embassy_time::{Duration, Ticker};

use crate::config::{CURRENT_COUNT, FORCE_COUNT, POSITION_COUNT};
use crate::protocol::build_telemetry_frame;
use crate::shared::SharedData;

const TELEM_MSG_HZ: u64 = 20;

#[task]
pub async fn telemetry_sender_task(
    mut tx: UartTx<'static, Async>,
    shared_positions: &'static SharedData<POSITION_COUNT>,
    shared_forces: &'static SharedData<FORCE_COUNT>,
    shared_currents: &'static SharedData<CURRENT_COUNT>,
) {
    let mut ticker = Ticker::every(Duration::from_hz(TELEM_MSG_HZ));

    loop {
        let positions = shared_positions.read_snapshot();
        let forces = shared_forces.read_snapshot();
        let currents = shared_currents.read_snapshot();
        let frame = build_telemetry_frame(positions, forces, currents);
        let _ = tx.write(&frame.buf[..frame.len]).await;
        // Fixed telemetry rate to PC.
        ticker.next().await;
    }
}
