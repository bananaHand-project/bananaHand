#![no_std]
#![no_main]

mod fmt;
mod c0_reader;
mod protocol;
mod shared_force;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::{
    Config,
    usart::Config as UartConfig,
};
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Timer};
use fmt::info;
use shared_force::SharedForceData;

bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

static SHARED_FORCE: SharedForceData = SharedForceData::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL85,
            divp: None,
            divq: None,
            // Main system clock at 170 MHz
            divr: Some(PllRDiv::DIV2),
        });
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.sys = Sysclk::PLL1_R;
        config.rcc.ls = LsConfig::default_lsi();
    }
    let p = embassy_stm32::init(config);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let uart_rx = UartRx::new(p.USART1, Irqs, p.PA10, p.DMA1_CH1, uart_config).unwrap();
    _spawner
        .spawn(c0_reader::c0_reader_task(uart_rx, &SHARED_FORCE))
        .unwrap();

    let mut latest = [0u16; 10];
    let mut read_count: u32 = 0;
    loop {
        if SHARED_FORCE.read_frame(&mut latest) {
            read_count = read_count.wrapping_add(1);
            if read_count == 1 || read_count % 10 == 0 {
                info!(
                    "Shared force read {}: ch0={} ch1={} ch2={} ch3={}",
                    read_count,
                    latest[0],
                    latest[1],
                    latest[2],
                    latest[3]
                );
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
