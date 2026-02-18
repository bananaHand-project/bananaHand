#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::{Duration, Timer};
use fmt::info;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PB6, Level::High, Speed::Low);
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let mut uart = Uart::new_blocking(p.USART1, p.PA10, p.PC14, uart_config).unwrap();

    const MSG: &[u8] = b"Hello from c0!\n";

    loop {
        let _ = uart.blocking_write(MSG);
        info!("Sent: Hello from c0!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
