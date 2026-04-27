# Firmware

Firmware for the BananaHand main board lives here. The board uses two MCUs:

- `stm32g474` (primary): communication, control loops, motor drive outputs.
- `stm32c071` (sensing): force sensor acquisition and streaming to the primary MCU.

## High-Level Architecture

- [`primary-g474/primary`](./primary-g474/primary): production firmware for the primary controller.
- [`sensing-c071`](./sensing-c071): production firmware for force sensing.
- [`common`](./common): shared packet definitions and encoding helpers.
- `experiments/`: one-off firmware targets used during bring-up and feature exploration.

## How It Works

1. The C071 samples 10 FSR channels (200 Hz), packs 12-bit readings with a checksum (`banana_hand_common::ForceDataPacket`), and sends them over UART to the G474.
2. The G474 runs concurrent Embassy tasks for host command RX (`USART3`), telemetry TX (`USART3`), force packet RX from C071 (`USART2`), position ADC sampling (`ADC1`), and current ADC sampling (`ADC2`).
3. A 200 Hz control loop on the G474 reads the latest shared snapshots and runs either `Position` mode (PID on actuator position) or `Force` mode (PID on force feedback where available).
4. Motor commands are converted to PWM outputs for the H-bridge channels.
5. Telemetry is sent to the host as combined position + force + current frames.

Current host communication is UART (`115200`) with COBS framing and a simple additive checksum on payloads.

## Build/Flash

Primary MCU (`STM32G474`):

```bash
cd firmware/primary-g474/primary
cargo run --release
```

Sensing MCU (`STM32C071`):

```bash
cd firmware/sensing-c071
cargo run --release
```

Both targets are configured to use `probe-rs run` in their local `.cargo/config.toml`.

## TODOs

- Add firmware support for CAN and half-duplex RS485 (board hardware already exposes both).
- Improve message passing robustness (stronger CRCs, sequence IDs with ACK/NACK or retries, and host heartbeat/command timeout fail-safe behavior).
- Remove unnecessary COBS usage (evaluate simpler framing on fixed-size links and delete stale code paths such as `sensing-c071/src/protocol.rs`).
- Tighten C071->G474 framing robustness (current stream has no explicit delimiter; resync is byte-sliding on checksum failure).
- Centralize protocol definitions so firmware and ROS bridge cannot drift.
- Create actuator/sensor calibration procedure (per-channel scaling and nonlinearity compensation).
- Add support for HRTIM triggered ADC conversions to `embassy-rs` so current feedback for protection and control (overcurrent limits, stall detection, fault handling).
