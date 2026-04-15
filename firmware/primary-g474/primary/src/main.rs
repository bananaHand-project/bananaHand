#![no_std]
#![no_main]

mod board_map;
mod c0_reader;
mod command_reader;
mod config;
mod control_config;
mod current_reader;
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
use embassy_stm32::time::khz;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcConfig},
    gpio::OutputType,
    rcc::{APBPrescaler, clocks},
    time::Hertz,
    timer::simple_pwm::{PwmPin, SimplePwm},
    usart::{Config as UartConfig, Uart, UartRx},
};
use embassy_time::{Duration, Ticker};
use hrtim_pwm_hal::{HrtimCore, HrtimPrescaler, period_reg_val};

use banana_hand_common::FORCE_SENS_BAUD;
use board_map::{MotorPwm, current_adc_pins, position_adc_pins};
use config::{COMMAND_COUNT, CURRENT_COUNT, FORCE_COUNT, POSITION_COUNT};
use control_config::{
    CONTROL_HZ, CommandInputs, ControlMode, DEFAULT_CONTROL_MODE, ForceInputs, PositionInputs,
};
use motor_control::{Controller, MotorCommand, MotorOutputs};
use protocol::TELEM_BAUD;
use shared::{SharedData, SharedMode};

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
    USART3 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART3>;
});

static SHARED_POSITIONS: SharedData<POSITION_COUNT> = SharedData::new();
static SHARED_CURRENTS: SharedData<CURRENT_COUNT> = SharedData::new();
static SHARED_POSITION_COMMANDS: SharedData<COMMAND_COUNT> = SharedData::new();
static SHARED_FORCE_COMMANDS: SharedData<COMMAND_COUNT> = SharedData::new();
static SHARED_CONTROL_MODE: SharedMode = SharedMode::new(DEFAULT_CONTROL_MODE.to_wire());
static SHARED_FORCE: SharedData<FORCE_COUNT> = SharedData::new();
const PWM_FREQ: Hertz = Hertz(20_000);

fn coast_all_outputs() -> MotorOutputs {
    MotorOutputs {
        ring: MotorCommand::Coast,
        pinky: MotorCommand::Coast,
        thumb_flex: MotorCommand::Coast,
        index1: MotorCommand::Coast,
        middle: MotorCommand::Coast,
        thumb_revolve: MotorCommand::Coast,
    }
}

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
    SHARED_CONTROL_MODE.write(DEFAULT_CONTROL_MODE.to_wire());

    // USART3: PC link (kept off PA2/PA3 because those are used as potentiometer inputs).
    let mut uart3_config = UartConfig::default();
    uart3_config.baudrate = TELEM_BAUD;
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
            &SHARED_CURRENTS,
        ))
        .unwrap();
    _spawner
        .spawn(command_reader::command_reader_task(
            rx3,
            &SHARED_POSITION_COMMANDS,
            &SHARED_FORCE_COMMANDS,
            &SHARED_CONTROL_MODE,
        ))
        .unwrap();

    // USART2: C0 force reader (RX only).
    let mut uart1_config = UartConfig::default();
    uart1_config.baudrate = FORCE_SENS_BAUD;

    // RX on PA15 from C0 USART1_TX (PB6).
    let uart1_rx = UartRx::new(p.USART2, Irqs, p.PA15, p.DMA1_CH1, uart1_config).unwrap();

    _spawner
        .spawn(c0_reader::c0_reader_task(uart1_rx, &SHARED_FORCE))
        .unwrap();

    // ADC + position reader (G4 pots).
    let dma = p.DMA1_CH2;
    let adc = Adc::new(p.ADC1, AdcConfig::default());
    let pos_pins = position_adc_pins(p.PA0, p.PA1, p.PA2, p.PA3, p.PB0, p.PB1, p.PB11, p.PF0);
    _spawner
        .spawn(position_reader::position_reader_task(
            adc,
            dma,
            pos_pins,
            &SHARED_POSITIONS,
        ))
        .unwrap();

    // ADC2 + current reader (same logical order as positions/commands).
    let current_dma = p.DMA2_CH1;
    let current_adc = Adc::new(p.ADC2, AdcConfig::default());
    let current_pins = current_adc_pins(p.PA4, p.PA5, p.PA6, p.PA7, p.PB2, p.PC4, p.PC5, p.PF1);
    _spawner
        .spawn(current_reader::current_reader_task(
            current_adc,
            current_dma,
            current_pins,
            &SHARED_CURRENTS,
        ))
        .unwrap();

    let prescaler = HrtimPrescaler::DIV32;
    let period = period_reg_val(clocks(&p.RCC), APBPrescaler::DIV1, prescaler, PWM_FREQ).unwrap();

    // TIM1 pin/channel contract:
    // CH1=PC0 and CH2=PC1 drive thumb-revolve H-bridge.
    // CH3=PC2 and CH4=PC3 are kept enabled but currently unused.
    let thumb_revolve_ch1_pin = PwmPin::new(p.PC0, OutputType::PushPull); // TIM1_CH1
    let thumb_revolve_ch2_pin = PwmPin::new(p.PC1, OutputType::PushPull); // TIM1_CH2
    let thumb_flex_unused_ch1_pin = PwmPin::new(p.PC2, OutputType::PushPull); // TIM1_CH3 (unused)
    let thumb_flex_unused_ch2_pin = PwmPin::new(p.PC3, OutputType::PushPull); // TIM1_CH4 (unused)
    let thumb_pwm = SimplePwm::new(
        p.TIM1,
        Some(thumb_revolve_ch1_pin),
        Some(thumb_revolve_ch2_pin),
        Some(thumb_flex_unused_ch1_pin),
        Some(thumb_flex_unused_ch2_pin),
        khz(20),
        Default::default(),
    );
    let thumb_pwm_ch = thumb_pwm.split();

    // HRTIM channels mapped directly to actuators.
    let (ring, pinky, thumb_flex, index1, middle, index2_unused) = HrtimCore::new()
        .add_tim_a_ch1_ch2(p.PA8, p.PA9, period, prescaler)
        .add_tim_b_ch1_ch2(p.PA10, p.PA11, period, prescaler)
        .add_tim_c_ch1_ch2(p.PB12, p.PB13, period, prescaler)
        .add_tim_d_ch1_ch2(p.PB14, p.PB15, period, prescaler)
        .add_tim_e_ch1_ch2(p.PC8, p.PC9, period, prescaler)
        .add_tim_f_ch1_ch2(p.PC6, p.PC7, period, prescaler)
        .split_active()
        .unwrap();

    let mut motor_pwm = MotorPwm::new(
        ring,
        pinky,
        thumb_flex,
        index1,
        middle,
        index2_unused,
        thumb_pwm_ch.ch1, // TIM1_CH1 -> PC0 -> thumb_revolve_ch1
        thumb_pwm_ch.ch2, // TIM1_CH2 -> PC1 -> thumb_revolve_ch2
        thumb_pwm_ch.ch3, // TIM1_CH3 -> PC2 -> unused
        thumb_pwm_ch.ch4, // TIM1_CH4 -> PC3 -> unused
    );

    let mut controller = Controller::default();
    let mut control_ticker = Ticker::every(Duration::from_hz(CONTROL_HZ));
    let mut applied_mode = DEFAULT_CONTROL_MODE;
    let mut waiting_for_fresh_command = false;
    let mut required_command_seq = SHARED_POSITION_COMMANDS.seq();

    // MAIN PID LOOP //
    loop {
        let requested_mode =
            ControlMode::from_wire(SHARED_CONTROL_MODE.read()).unwrap_or(DEFAULT_CONTROL_MODE);

        if requested_mode != applied_mode {
            let latest_positions = SHARED_POSITIONS.read_snapshot();
            let latest_force = SHARED_FORCE.read_snapshot();
            let latest_position_commands = SHARED_POSITION_COMMANDS.read_snapshot();
            let latest_force_commands = SHARED_FORCE_COMMANDS.read_snapshot();

            defmt::info!(
                "mode change: {} -> {}, position_cmds: {}, force_cmds: {}, positions: {}, forces: {}",
                applied_mode.to_wire(),
                requested_mode.to_wire(),
                latest_position_commands,
                latest_force_commands,
                latest_positions,
                latest_force
            );

            controller.set_mode(requested_mode);
            applied_mode = requested_mode;
            waiting_for_fresh_command = true;
            required_command_seq = match applied_mode {
                ControlMode::Position => SHARED_POSITION_COMMANDS.seq(),
                ControlMode::Force => SHARED_FORCE_COMMANDS.seq(),
            };
        }

        let latest_force = SHARED_FORCE.read_snapshot();
        let latest_positions = SHARED_POSITIONS.read_snapshot();
        let position_commands = SHARED_POSITION_COMMANDS.read_snapshot();
        let force_commands = SHARED_FORCE_COMMANDS.read_snapshot();

        let active_command_seq = match applied_mode {
            ControlMode::Position => SHARED_POSITION_COMMANDS.seq(),
            ControlMode::Force => SHARED_FORCE_COMMANDS.seq(),
        };

        if waiting_for_fresh_command && active_command_seq != required_command_seq {
            waiting_for_fresh_command = false;
        }

        let active_commands = match applied_mode {
            ControlMode::Position => &position_commands,
            ControlMode::Force => &force_commands,
        };

        let command_inputs = CommandInputs::from_raw(active_commands);
        let position_inputs = PositionInputs::from_raw(&latest_positions);
        let force_inputs = ForceInputs::from_raw(&latest_force);

        let outputs = if waiting_for_fresh_command {
            coast_all_outputs()
        } else {
            controller.step(&command_inputs, &position_inputs, &force_inputs)
        };
        let pwm_outputs = outputs.into_pwm_outputs();
        motor_pwm.apply_outputs(pwm_outputs);

        control_ticker.next().await;
    }
}
