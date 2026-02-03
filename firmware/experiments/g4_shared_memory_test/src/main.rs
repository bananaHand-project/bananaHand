#![no_std]
#![no_main]

mod fmt;
mod c0_reader;
mod protocol;
mod shared_data;
mod position_reader;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::{
    Config,
    adc::{Adc, SampleTime},
    usart::Config as UartConfig,
};
use embassy_stm32::usart::UartRx;
use embassy_time::{Duration, Timer};
use embassy_stm32::adc::AdcChannel;
use fmt::info;
use shared_data::SharedData;

bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
});

static SHARED_FORCE: SharedData<10> = SharedData::new();
static SHARED_POSITION: SharedData<8> = SharedData::new();

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

    // ADC + position reader (G4 pots)
    let dma = p.DMA1_CH2;
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES640_5);
    let pos_ch = [
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
        p.PA2.degrade_adc(),
        p.PA3.degrade_adc(),
        p.PF0.degrade_adc(),
        p.PB1.degrade_adc(),
        p.PB11.degrade_adc(),
        p.PB0.degrade_adc(),
    ];
    _spawner
        .spawn(position_reader::position_reader_task(adc, dma, pos_ch, &SHARED_POSITION))
        .unwrap();

    let mut latest = [0u16; 10];
    let mut pos_latest = [0u16; 8];
    let mut read_count: u32 = 0;
    let mut pos_read_count: u32 = 0;
    loop {
        if SHARED_FORCE.read_frame(&mut latest) {
            read_count = read_count.wrapping_add(1);
            if read_count == 1 || read_count % 10 == 0 {
                // info!(
                //     "Shared force read {}: ch0={} ch1={} ch2={} ch3={} ch4={} ch5={} ch6={} ch7={} ch8={} ch9={}",
                //     read_count,
                //     latest[0],
                //     latest[1],
                //     latest[2],
                //     latest[3],
                //     latest[4],
                //     latest[5],
                //     latest[6],
                //     latest[7],
                //     latest[8],
                //     latest[9],
                // );
            }
        }
        if SHARED_POSITION.read_frame(&mut pos_latest) {
            pos_read_count = pos_read_count.wrapping_add(1);
            if pos_read_count == 1 || pos_read_count % 10 == 0 {
                info!(
                    "Shared pos read {}: ch0={} ch1={} ch2={} ch3={} ch4={} ch5={} ch6={} ch7={}",
                    pos_read_count,
                    pos_latest[0],
                    pos_latest[1],
                    pos_latest[2],
                    pos_latest[3],
                    pos_latest[4],
                    pos_latest[5],
                    pos_latest[6],
                    pos_latest[7],
                );
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
