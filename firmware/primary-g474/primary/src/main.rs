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
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx};
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel, AdcConfig},
    gpio::OutputType,
    rcc::{APBPrescaler, clocks},
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
};
use embassy_stm32::{bind_interrupts, time::khz};
use embassy_time::{Duration, Ticker};
use hrtim_pwm_hal::{HrtimCore, HrtimPrescaler, period_reg_val};

use config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};
use control_config::{
    CONTROL_HZ, MAX_MOTORS, MOTOR_INDEX_1, MOTOR_MIDDLE, MOTOR_NAMES, MOTOR_PINKY, MOTOR_RING,
    MOTOR_THUMB_1, MOTOR_THUMB_2, POS_THUMB_2, POS_THUMB_3, POSITION_NAMES, is_valid_config,
};
use motor_control::{Controller, MotorPwmCommand, pwm_command_from_motor_command};
use shared::SharedData;

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

static SHARED_POSITIONS: SharedData<POSITION_COUNT> = SharedData::new();
static SHARED_COMMANDS: SharedData<COMMAND_COUNT> = SharedData::new();
static SHARED_FORCE: SharedData<FORCE_COUNT> = SharedData::new();
const PWM_FREQ: Hertz = Hertz(20_000);

fn swap_pwm_channels_if_req(motor_idx: usize, mut cmd: MotorPwmCommand) -> MotorPwmCommand {
    if control_config::MOTOR_PWM_SWAP[motor_idx] {
        core::mem::swap(&mut cmd.ch1_percent, &mut cmd.ch2_percent);
    }
    cmd
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // debug_assert!(is_valid_config());
    // debug_assert!(MOTOR_NAMES.len() == MAX_MOTORS);
    // debug_assert!(POSITION_NAMES.len() == POSITION_COUNT);
    // debug_assert!(POS_THUMB_2 == 6 && POS_THUMB_3 == 7);

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
        p.DMA1_CH6, // TX DMA channel
        p.DMA1_CH5, // RX DMA channel
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
    let _uart1_rx = UartRx::new(p.USART2, Irqs, p.PA15, p.DMA1_CH1, uart1_config).unwrap();

    _spawner
        .spawn(c0_reader::c0_reader_task(_uart1_rx, &SHARED_FORCE))
        .unwrap();

    // ADC + position reader (G4 pots).
    let dma = p.DMA1_CH2;
    let adc = Adc::new(p.ADC1, AdcConfig::default());
    // Keep ADC channel order aligned with control_config::POS_* constants.
    let pos_ch = [
        p.PA3.degrade_adc(),  // POS_INDEX_1
        p.PB1.degrade_adc(),  // POS_MIDDLE
        p.PB0.degrade_adc(),  // POS_RING
        p.PF0.degrade_adc(),  // POS_PINKY
        p.PA2.degrade_adc(),  // POS_THUMB_1 (flex)
        p.PA0.degrade_adc(),  // POS_THUMB_2 (revolve)
        p.PB11.degrade_adc(), // POS_INDEX_2 (USELESS)
        p.PA1.degrade_adc(),  // POS_THUMB_3 (USELESS)
    ];
    _spawner
        .spawn(position_reader::position_reader_task(
            adc,
            dma,
            pos_ch,
            &SHARED_POSITIONS,
        ))
        .unwrap();

    // OTHER PWM TIMERS
    let thumb_rev_ch1 = PwmPin::new(p.PC0, OutputType::PushPull);
    let thumb_rev_ch2 = PwmPin::new(p.PC1, OutputType::PushPull);
    let thumb_flex_ch1 = PwmPin::new(p.PC2, OutputType::PushPull);
    let thumb_flex_ch2 = PwmPin::new(p.PC3, OutputType::PushPull);
    let thumb_pwm = SimplePwm::new(
        p.TIM1,
        Some(thumb_rev_ch1),
        Some(thumb_rev_ch2),
        Some(thumb_flex_ch1),
        Some(thumb_flex_ch2),
        khz(20),
        Default::default(),
    );
    let thumb_pwm_ch = thumb_pwm.split();
    let mut thumb_rev_ch1 = thumb_pwm_ch.ch1; // THUMB REVOLVE
    let mut thumb_rev_ch2 = thumb_pwm_ch.ch2; // THUMB REVOLVE
    let mut thumb_flex_ch1 = thumb_pwm_ch.ch3; // (USELESS)
    let mut thumb_flex_ch2 = thumb_pwm_ch.ch4; // (USELESS)

    thumb_rev_ch1.enable(); // THUMB REVOLVE
    thumb_rev_ch2.enable(); // THUMB REVOLVE
    thumb_flex_ch1.enable(); // (USELESS)
    thumb_flex_ch2.enable(); // (USELESS)

    // HRTIM PWM TIMERS
    let prescaler = HrtimPrescaler::DIV32;
    let period = period_reg_val(clocks(&p.RCC), APBPrescaler::DIV1, prescaler, PWM_FREQ).unwrap();
    let (tim_a, tim_b, tim_c, tim_d, tim_e, tim_f) = HrtimCore::new()
        .add_tim_a_ch1_ch2(p.PA8, p.PA9, period, prescaler) // RING
        .add_tim_b_ch1_ch2(p.PA10, p.PA11, period, prescaler) // PINKY
        .add_tim_c_ch1_ch2(p.PB12, p.PB13, period, prescaler) // THUMB FLEX
        .add_tim_d_ch1_ch2(p.PB14, p.PB15, period, prescaler) // INDEX 1 !!
        .add_tim_e_ch1_ch2(p.PC8, p.PC9, period, prescaler) // MIDDLE
        .add_tim_f_ch1_ch2(p.PC6, p.PC7, period, prescaler) // INDEX 2!! (USELESS)
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
    // let mut command_toggle = true;

    loop {
        let mut latest_cmd = SHARED_COMMANDS.read_snapshot();

        // IF YOU WANT TO SEND MANUAL HARD CODED COMMANDS
        // status_tick = status_tick.wrapping_add(1);
        // if status_tick % 200 == 0 {

        //     command_toggle = !command_toggle;
        // }
        // if (command_toggle) {
        //     latest_cmd =
        //     [
        //         0,
        //         0,
        //         0,
        //         0,
        //         0,
        //         0,
        //         0,
        //         0
        //     ]
        // } else {
        //     latest_cmd =
        //     [
        //         0,
        //         0,
        //         0,
        //         0,
        //         0,
        //         4000,
        //         0,
        //         0
        //     ]
        // };

        let latest_force = SHARED_FORCE.read_snapshot();
        let latest_positions = SHARED_POSITIONS.read_snapshot();

        defmt::info!(
            "command: {}, positions: {}, forces: {}",
            latest_cmd,
            latest_positions,
            latest_force
        );

        controller.set_mode(control_config::CONTROL_MODE);
        let outputs = controller.step(&latest_cmd, &latest_positions, &latest_force);
        let m_ring = pwm_command_from_motor_command(outputs[MOTOR_RING]);
        let m_pinky = pwm_command_from_motor_command(outputs[MOTOR_PINKY]);
        let m_thumb_1 = pwm_command_from_motor_command(outputs[MOTOR_THUMB_1]);
        let m_index_1 = pwm_command_from_motor_command(outputs[MOTOR_INDEX_1]);
        let m_middle = pwm_command_from_motor_command(outputs[MOTOR_MIDDLE]);
        let m_thumb_2 = pwm_command_from_motor_command(outputs[MOTOR_THUMB_2]);

        let m_ring = swap_pwm_channels_if_req(MOTOR_RING, m_ring);
        let m_pinky = swap_pwm_channels_if_req(MOTOR_PINKY, m_pinky);
        let m_thumb_1 = swap_pwm_channels_if_req(MOTOR_THUMB_1, m_thumb_1);
        let m_index_1 = swap_pwm_channels_if_req(MOTOR_INDEX_1, m_index_1);
        let m_middle = swap_pwm_channels_if_req(MOTOR_MIDDLE, m_middle);
        let m_thumb_2 = swap_pwm_channels_if_req(MOTOR_THUMB_2, m_thumb_2);

        tim_a.ch1_set_dc_percent(m_ring.ch1_percent);
        tim_a.ch2_set_dc_percent(m_ring.ch2_percent);
        tim_b.ch1_set_dc_percent(m_pinky.ch1_percent);
        tim_b.ch2_set_dc_percent(m_pinky.ch2_percent);
        tim_c.ch1_set_dc_percent(m_thumb_1.ch1_percent);
        tim_c.ch2_set_dc_percent(m_thumb_1.ch2_percent);
        tim_d.ch1_set_dc_percent(m_index_1.ch1_percent);
        tim_d.ch2_set_dc_percent(m_index_1.ch2_percent);
        tim_e.ch1_set_dc_percent(m_middle.ch1_percent);
        tim_e.ch2_set_dc_percent(m_middle.ch2_percent);
        thumb_rev_ch1.set_duty_cycle_percent(m_thumb_2.ch1_percent);
        thumb_rev_ch2.set_duty_cycle_percent(m_thumb_2.ch2_percent);

        control_ticker.next().await;
    }
}
