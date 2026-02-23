#![no_std]
#![no_main]

mod c0_reader;
mod command_reader;
mod config;
mod fmt;
mod position_reader;
mod protocol;
mod shared;
mod telemetry_sender;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::bind_interrupts;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx};
use embassy_stm32::{Config, adc::{Adc, SampleTime, AdcChannel}};
use embassy_time::{Duration, Timer};

use config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};
use shared::SharedData;

bind_interrupts!(struct Irqs {
    USART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART1>;
    LPUART1 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::LPUART1>;
});

static SHARED_POSITIONS: SharedData<POSITION_COUNT> = SharedData::new();
static SHARED_COMMANDS: SharedData<COMMAND_COUNT> = SharedData::new();
static SHARED_FORCE: SharedData<FORCE_COUNT> = SharedData::new();

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

    // LPUART1: PC link over STLINK VCP on NUCLEO-G474RE default routing (PA2/PA3).
    let mut uart3_config = UartConfig::default();
    uart3_config.baudrate = 115_200;
    let uart3 = Uart::new(
        p.LPUART1,
        p.PA3, // RX pin (PC -> MCU)
        p.PA2, // TX pin (MCU -> PC)
        Irqs,
        p.DMA1_CH6,  // TX DMA channel
        p.DMA1_CH5,  // RX DMA channel
        uart3_config,
    )
    .unwrap();

    let (tx3, rx3) = uart3.split();
    _spawner
        .spawn(telemetry_sender::telemetry_sender_task(
            tx3,
            &SHARED_POSITIONS,
            &SHARED_FORCE,
        ))
        .unwrap();
    _spawner
        .spawn(command_reader::command_reader_task(rx3, &SHARED_COMMANDS))
        .unwrap();

    // USART1: C0 force reader (RX only).
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = 115_200;

    // RX on PA10 from C0 USART1_TX (PB6).
    let uart1_rx = UartRx::new(p.USART1, Irqs, p.PA10, p.DMA1_CH1, uart1_config).unwrap();
    _spawner
        .spawn(c0_reader::c0_reader_task(uart1_rx, &SHARED_FORCE))
        .unwrap();

    // ADC + position reader (G4 pots).
    // NOTE: Avoid PA2/PA3 reserved for STLINK VCP (LPUART1 default on MB1367).
    let dma = p.DMA1_CH2;
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES640_5);
    // Keep this list aligned with POSITION_COUNT.
    let pos_ch = [
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
        p.PB0.degrade_adc(),
        p.PB1.degrade_adc(),
        p.PC0.degrade_adc(),
        p.PC1.degrade_adc(),
        p.PC2.degrade_adc(),
        p.PC3.degrade_adc(),
    ];
    _spawner
        .spawn(position_reader::position_reader_task(adc, dma, pos_ch, &SHARED_POSITIONS))
        .unwrap();

    let mut latest_cmd = [0u16; COMMAND_COUNT];
    let mut latest_force = [0u16; FORCE_COUNT];
    let mut latest_positions = [0u16; POSITION_COUNT];
    loop {
        if SHARED_COMMANDS.read_frame(&mut latest_cmd) {
            defmt::info!("Commands: {:?}", latest_cmd);
            defmt::info!("Commands updated (seq={})", SHARED_COMMANDS.seq());
        }
        // if SHARED_POSITIONS.read_frame(&mut latest_positions) {
        //     defmt::info!("Positions: {:?}", latest_positions);
        // }
        // if SHARED_FORCE.read_frame(&mut latest_force) {
        //     defmt::info!("Force values: {:?}", latest_force);
        //     defmt::info!("Force updated (seq={})", SHARED_FORCE.seq());
        // }
        Timer::after(Duration::from_millis(200)).await;
    }
}
