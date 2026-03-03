#![no_std]
#![no_main]

mod c0_reader;
mod command_reader;
mod config;
mod control_config;
mod fmt;
mod motor_control;
mod pid;
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
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel, AdcConfig},
    rcc::{APBPrescaler, clocks},
    time::Hertz,
};
use embassy_time::{Duration, Ticker};
use hrtim_pwm_hal::{HrtimCore, HrtimPrescaler, period_reg_val};

use config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};
use control_config::{CONTROL_HZ, is_valid_config};
use motor_control::{Controller, pwm_command_from_motor_command};
use shared::SharedData;

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

static SHARED_POSITIONS: SharedData<POSITION_COUNT> = SharedData::new();
static SHARED_COMMANDS: SharedData<COMMAND_COUNT> = SharedData::new();
static SHARED_FORCE: SharedData<FORCE_COUNT> = SharedData::new();
const PWM_FREQ: Hertz = Hertz(20_000);

fn control_mode_label() -> &'static str {
    match control_config::CONTROL_MODE {
        control_config::ControlMode::Position => "position",
        control_config::ControlMode::Force => "force",
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    debug_assert!(is_valid_config());

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

    // USART3: PC link (kept off PA2/PA3 because those are used as potentiometer inputs).
    let mut uart3_config = UartConfig::default();
    uart3_config.baudrate = 115_200;
    let uart3 = Uart::new(
        p.USART3,
        p.PC11, // RX pin (PC -> MCU)
        p.PB10, // TX pin (MCU -> PC)
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

    // USART2: C0 force reader (RX only).
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = 115_200;

    // RX on PA15 from C0 USART1_TX (PB6).
    let uart1_rx = UartRx::new(p.USART2, Irqs, p.PA15, p.DMA1_CH1, uart1_config).unwrap();
    _spawner
        .spawn(c0_reader::c0_reader_task(uart1_rx, &SHARED_FORCE))
        .unwrap();

    // ADC + position reader (G4 pots).
    let dma = p.DMA1_CH2;
    let adc = Adc::new(p.ADC1, AdcConfig::default());
    // Position channels in motor order for the first six motors:
    let pos_ch = [
        p.PB0.degrade_adc(),
        p.PF0.degrade_adc(),
        p.PA2.degrade_adc(), // should be smth else?
        p.PA3.degrade_adc(), // INDEX 1 !!
        p.PB1.degrade_adc(),
        p.PB11.degrade_adc(), // INDEX 2!!
        p.PC2.degrade_adc(), // should be pa1 or pa2?
        p.PC3.degrade_adc(), // should be pa1 or pa2?
    ];
    _spawner
        .spawn(position_reader::position_reader_task(adc, dma, pos_ch, &SHARED_POSITIONS))
        .unwrap();

    let prescaler = HrtimPrescaler::DIV32;
    let period = period_reg_val(clocks(&p.RCC), APBPrescaler::DIV1, prescaler, PWM_FREQ).unwrap();
    let (tim_a, tim_b, tim_c, tim_d, tim_e, tim_f) = HrtimCore::new()
        .add_tim_a_ch1_ch2(p.PA8, p.PA9, period, prescaler)
        .add_tim_b_ch1_ch2(p.PA10, p.PA11, period, prescaler)
        .add_tim_c_ch1_ch2(p.PB12, p.PB13, period, prescaler)
        .add_tim_d_ch1_ch2(p.PB14, p.PB15, period, prescaler) // INDEX 1 !!
        .add_tim_e_ch1_ch2(p.PC8, p.PC9, period, prescaler)
        .add_tim_f_ch1_ch2(p.PC6, p.PC7, period, prescaler) // INDEX 2!!
        .split_active()
        .unwrap();
    tim_a.ch1_en();
    tim_a.ch2_en();
    tim_b.ch1_en();
    tim_b.ch2_en();
    tim_c.ch1_en();
    tim_c.ch2_en();
    tim_d.ch1_en();
    tim_d.ch2_en();
    tim_e.ch1_en();
    tim_e.ch2_en();
    tim_f.ch1_en();
    tim_f.ch2_en();

    let mut controller = Controller::new();
    let mut control_ticker = Ticker::every(Duration::from_hz(CONTROL_HZ));
    let mut status_tick: u32 = 0;

    // MAIN PID LOOP //

    loop {
        let mut latest_cmd = SHARED_COMMANDS.read_snapshot();

        latest_cmd = [
            0,
            0,
            0,
            0,
            0,
            4000,
            0,
            0
        ];

        
        let latest_force = SHARED_FORCE.read_snapshot();
        let latest_positions = SHARED_POSITIONS.read_snapshot();

        // defmt::info!("position: {}",latest_positions);

        controller.set_mode(control_config::CONTROL_MODE);
        let outputs = controller.step(&latest_cmd, &latest_positions, &latest_force);
        let m0 = pwm_command_from_motor_command(outputs[0]);
        let m1 = pwm_command_from_motor_command(outputs[1]);
        let m2 = pwm_command_from_motor_command(outputs[2]);
        let m3 = pwm_command_from_motor_command(outputs[3]);
        let m4 = pwm_command_from_motor_command(outputs[4]);
        let m5 = pwm_command_from_motor_command(outputs[5]);
        defmt::info!("position: {}, 3Dch1: {}, 3Dch2: {}, 5Fch1: {}, 5Fch2: {}", latest_positions, m3.ch1_percent, m3.ch2_percent, m5.ch1_percent, m5.ch2_percent);

        tim_a.ch1_set_dc_percent(m0.ch1_percent);
        tim_a.ch2_set_dc_percent(m0.ch2_percent);
        tim_b.ch1_set_dc_percent(m1.ch1_percent);
        tim_b.ch2_set_dc_percent(m1.ch2_percent);
        tim_c.ch1_set_dc_percent(m2.ch1_percent);
        tim_c.ch2_set_dc_percent(m2.ch2_percent);

        // changed from m3
        tim_d.ch1_set_dc_percent(m3.ch2_percent); // OPPOSITE!! (index 1)
        tim_d.ch2_set_dc_percent(m3.ch1_percent); // OPPOSITE!! (index 1)

        tim_e.ch1_set_dc_percent(m4.ch1_percent);
        tim_e.ch2_set_dc_percent(m4.ch2_percent);

        // changed from m5
        tim_f.ch1_set_dc_percent(m5.ch1_percent);
        tim_f.ch2_set_dc_percent(m5.ch2_percent);

        // status_tick = status_tick.wrapping_add(1);
        // if status_tick % 40 == 0 {
        //     defmt::info!(
        //         "mode={} cmd0={} pos0={} force0={} seq(c/p/f)=({}/{}/{})",
        //         control_mode_label(),
        //         latest_cmd[0],
        //         latest_positions[0],
        //         latest_force[0],
        //         SHARED_COMMANDS.seq(),
        //         SHARED_POSITIONS.seq(),
        //         SHARED_FORCE.seq(),
        //     );
        // }

        control_ticker.next().await;
    }
}
