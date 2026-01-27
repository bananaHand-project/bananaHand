#![no_std]
#![no_main]

mod protocol;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{Config, Uart};
use protocol::{build_frame, FrameParser, MessageType};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 115_200;

    let mut uart = Uart::new(
        p.USART2,
        p.PA3,       // RX pin
        p.PA2,       // TX pin
        Irqs,
        p.DMA1_CH6,  // TX DMA channel
        p.DMA1_CH5,  // RX DMA channel
        config,
    )
    .unwrap();

    defmt::info!("UART2 initialized (COBS framing)!");

    // Raw bytes read from DMA
    let mut rx_buf = [0u8; 64];

    // COBS streaming parser
    let mut parser = FrameParser::new();

    loop {
        // Read a burst of bytes (until idle) into rx_buf
        let n = uart.read_until_idle(&mut rx_buf).await.unwrap();

        for &b in &rx_buf[..n] {
            // parse_byte returns (msg_type, payload_slice)
            if let Some((msg_type, payload)) = parser.parse_byte(b) {
                if msg_type == MessageType::PositionUpdate as u8 {
                    // Expect 8 f32 = 32 bytes
                    if payload.len() != 32 {
                        defmt::warn!("PositionUpdate payload wrong size: {}", payload.len());
                        continue;
                    }

                    let mut positions = [0f32; 8];
                    for i in 0..8 {
                        let start = i * 4;
                        let bytes: [u8; 4] = payload[start..start + 4].try_into().unwrap();
                        positions[i] = f32::from_le_bytes(bytes);
                        defmt::info!("pos[{}] = {}", i, positions[i]);
                    }

                    // Echo back using COBS framed builder (returns {buf, len})
                    let frame = build_frame(MessageType::PositionUpdate as u8, positions);
                    let _ = uart.write(&frame.buf[..frame.len]).await;
                } else {
                    defmt::info!("Unhandled message type {}", msg_type);
                }
            }
        }
    }
}
