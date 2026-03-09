#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_time::Timer;

use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);
    // let mut uart_cfg = UartConfig::default();
    // uart_cfg.baudrate = 115_200;
    // let mut uart_rx = UartRx::new_blocking(p.USART3, p.PC11, uart_cfg).unwrap();
    // let mut byte = [0u8; 1];

    loop {
        
        info!("hello");
        // if uart_rx.blocking_read(&mut byte).is_err() {
        //     continue;
        // }
        // info!("RX byte: 0x{:02x} ({})", byte[0], byte[0]);
        led.toggle();
        Timer::after_secs(1).await;
    }
}
