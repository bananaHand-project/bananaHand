use embassy_executor::task;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::UartTx;
use embassy_time::{Duration, Ticker, with_timeout};

use crate::config::{CURRENT_COUNT, FORCE_COUNT, POSITION_COUNT};
use crate::protocol::build_telemetry_frame;
use crate::shared::SharedData;

pub const TELEM_TX_HZ: u64 = 20;
const TX_TIMEOUT_MS: u64 = 15;
const TX_MAX_ATTEMPTS: usize = 3;

async fn send_frame_with_timeout_retries(tx: &mut UartTx<'static, Async>, frame: &[u8]) {
    for attempt in 1..=TX_MAX_ATTEMPTS {
        // In embassy-stm32 v0.5.0 this TX path is effectively infallible on Result
        // (write/flush currently return Ok(())); timeout-based retry is used so a stalled
        // DMA/interrupt path does not block telemetry forever.
        let write_completed =
            with_timeout(Duration::from_millis(TX_TIMEOUT_MS), tx.write(frame)).await;
        let Ok(write_result) = write_completed else {
            defmt::warn!(
                "telemetry write timeout on attempt {}/{}",
                attempt,
                TX_MAX_ATTEMPTS
            );
            continue;
        };

        if write_result.is_err() {
            defmt::warn!(
                "telemetry write returned error on attempt {}/{}",
                attempt,
                TX_MAX_ATTEMPTS
            );
            continue;
        }

        let flush_completed = with_timeout(Duration::from_millis(TX_TIMEOUT_MS), tx.flush()).await;
        let Ok(flush_result) = flush_completed else {
            defmt::warn!(
                "telemetry flush timeout on attempt {}/{}",
                attempt,
                TX_MAX_ATTEMPTS
            );
            continue;
        };

        if flush_result.is_err() {
            defmt::warn!(
                "telemetry flush returned error on attempt {}/{}",
                attempt,
                TX_MAX_ATTEMPTS
            );
            continue;
        }

        return;
    }

    defmt::error!(
        "telemetry send failed after {} timeout-based attempts",
        TX_MAX_ATTEMPTS
    );
}

#[task]
pub async fn telemetry_sender_task(
    mut tx: UartTx<'static, Async>,
    shared_positions: &'static SharedData<POSITION_COUNT>,
    shared_forces: &'static SharedData<FORCE_COUNT>,
    shared_currents: &'static SharedData<CURRENT_COUNT>,
) {
    let mut ticker = Ticker::every(Duration::from_hz(TELEM_TX_HZ));

    loop {
        let positions = shared_positions.read_snapshot();
        let forces = shared_forces.read_snapshot();
        let currents = shared_currents.read_snapshot();
        let frame = build_telemetry_frame(positions, forces, currents);

        send_frame_with_timeout_retries(&mut tx, &frame.buf[..frame.len]).await;

        ticker.next().await;
    }
}
