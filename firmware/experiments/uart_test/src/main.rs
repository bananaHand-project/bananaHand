#![no_std]
#![no_main]

mod protocol;
mod shared;
mod command_reader;
mod telemetry_sender;

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{Config, Uart};
use embassy_time::Timer;
use shared::{COMMANDS, POSITIONS};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut config = Config::default();
    config.baudrate = 115_200;

    let uart = Uart::new(
        p.USART2,
        p.PA3,       // RX pin
        p.PA2,       // TX pin
        Irqs,
        p.DMA1_CH6,  // TX DMA channel
        p.DMA1_CH5,  // RX DMA channel
        config,
    )
    .unwrap();

    let (tx, rx) = uart.split();

    _spawner.spawn(telemetry_sender::telemetry_sender_task(tx)).unwrap();
    _spawner.spawn(command_reader::command_reader_task(rx)).unwrap();

    loop {
        let positions = POSITIONS.read_snapshot();
        for (idx, value) in positions.iter().enumerate() {
            defmt::info!("positions[{}] = {}", idx, value);
        }

        let commands = COMMANDS.read_snapshot();
        for (idx, value) in commands.iter().enumerate() {
            defmt::info!("commands[{}] = {}", idx, value);
        }

        Timer::after_millis(500).await;
    }
}
