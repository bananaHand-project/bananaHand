#![no_std]
#![no_main]

mod protocol;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{Config, Uart};
use protocol::{FrameParser, build_frame, MessageType};
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
        p.PA3, // RX pin
        p.PA2, // TX pin
        Irqs, 
        p.DMA1_CH6, // TX DMA channel
        p.DMA1_CH5, // RX DMA channel
        config,
    ).unwrap();

    defmt::info!("UART2 initialized!");

    let mut buf = [0u8; 64];
    let mut parser = FrameParser::new();

    loop {
        let n = uart.read_until_idle(&mut buf).await.unwrap();

        for &b in &buf[..n] {
            if let Some((msg_type, data)) = parser.parse_byte(b) {
                // defmt::info!("Got frame type={} len={}", msg_type, data.len());

                // If this is a position update frame, try to interpret payload as 8 little-endian f32s
                if msg_type == MessageType::PositionUpdate as u8 {
                    let mut positions = [0f32; 8];
                    let count = core::cmp::min(8, data.len() / 4);
                    for i in 0..count {
                        let start = i * 4;
                        let bytes: [u8; 4] = [data[start], data[start + 1], data[start + 2], data[start + 3]];
                        positions[i] = f32::from_le_bytes(bytes);
                        defmt::info!(" pos[{}] = {}", i, positions[i]);
                    }

                    // Echo the positions back using build_frame
                    let frame = build_frame(MessageType::PositionUpdate as u8, positions);
                    let _ = uart.write(frame.as_slice()).await;
                } else {
                    defmt::info!("Unhandled message type {}", msg_type);
                }
            }
        }
    }
}
