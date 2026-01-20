#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel, SampleTime},
    gpio::{Level, Output, OutputType, Speed},
    peripherals::ADC1,
    time::Hertz,
    timer::{
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm},
    },
    usart::{Config as UartConfig, UartRx},
};
use embassy_stm32::mode::Blocking;
use embassy_time::{Duration, Ticker};
use fmt::info;
use pq12p_actuator::Pq12P;

const TARGET_FORCE_G: f32 = 150.0;
const FORCE_DEADBAND_G: f32 = 30.0;
const CONTROL_LOOP_HZ: u64 = 50;

const KP: f32 = 0.5;
const KI: f32 = 0.0;
const KD: f32 = 0.0;
const INTEGRAL_LIMIT: f32 = 500.0;
const DERIVATIVE_FILTER_TAU_S: f32 = 0.5;

const DUTY_MIN_PERCENT: f32 = 20.0; // stiction
const DUTY_MAX_PERCENT: f32 = 100.0;

const UART_LINE_MAX: usize = 64;

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
    }
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES640_5);

    let pwm = SimplePwm::new(
        p.TIM3,
        Some(PwmPin::new(p.PA6, OutputType::PushPull)),
        Some(PwmPin::new(p.PC7, OutputType::PushPull)),
        None,
        None,
        Hertz(20_000),
        CountingMode::EdgeAlignedUp,
    );
    let mut pwm_channels = pwm.split();
    pwm_channels.ch1.enable();
    pwm_channels.ch2.enable();

    let mut actuator: Pq12P<'_, embassy_stm32::peripherals::TIM3, ADC1> = Pq12P::new(
        pwm_channels.ch1,
        pwm_channels.ch2,
        p.PA0.degrade_adc(),
        p.PA1.degrade_adc(),
    );

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let mut uart_rx = UartRx::new_blocking(p.USART1, p.PA10, uart_config).unwrap();

    let mut ticker = Ticker::every(Duration::from_hz(CONTROL_LOOP_HZ));
    let dt = 1.0 / CONTROL_LOOP_HZ as f32;
    let derivative_alpha = dt / (DERIVATIVE_FILTER_TAU_S + dt);
    let mut integral = 0.0;
    let mut prev_error = 0.0;
    let mut filtered_derivative = 0.0;
    let mut current_force = None;

    let mut line_buf = [0u8; UART_LINE_MAX];

    loop {
        let line_len = read_line_blocking(&mut uart_rx, &mut line_buf);
        if let Some(force) = parse_force_line(&line_buf[..line_len]) {
            current_force = Some(force);
        }

        let Some(force) = current_force else {
            actuator.coast();
            ticker.next().await;
            continue;
        };

        let error = TARGET_FORCE_G - force;
        let mut control: f32 = 0.0;
        if error.abs() <= FORCE_DEADBAND_G {
            actuator.coast();
            integral = 0.0;
            prev_error = error;
            filtered_derivative = 0.0;
        } else {
            integral = (integral + error * dt).clamp(-INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            let raw_derivative = (error - prev_error) / dt;
            filtered_derivative += derivative_alpha * (raw_derivative - filtered_derivative);
            control = KP * error + KI * integral + KD * filtered_derivative;
            prev_error = error;

            let duty = control.abs().clamp(DUTY_MIN_PERCENT, DUTY_MAX_PERCENT);
            let duty_u8 = duty as u8;

            if control > 0.0 {
                actuator.move_out(duty_u8);
            } else {
                actuator.move_in(duty_u8);
            }
        }

        // info!("Force (uart): {} g", force);
        info!("Control: {}", control);
        led.toggle();
        ticker.next().await;
    }
}

fn read_line_blocking(rx: &mut UartRx<'_, Blocking>, buf: &mut [u8]) -> usize {
    let mut idx = 0;
    loop {
        let mut byte = [0u8; 1];
        if rx.blocking_read(&mut byte).is_err() {
            continue;
        }
        let ch = byte[0];
        if ch == b'\n' {
            break;
        }
        if idx < buf.len() {
            buf[idx] = ch;
            idx += 1;
        }
    }
    if idx > 0 && buf[idx - 1] == b'\r' {
        idx -= 1;
    }
    idx
}

fn parse_force_line(line: &[u8]) -> Option<f32> {
    const PREFIX: &[u8] = b"force_g=";
    if !line.starts_with(PREFIX) {
        return None;
    }
    let mut i = PREFIX.len();
    let mut whole = 0u32;
    let mut saw_digit = false;
    while i < line.len() {
        let b = line[i];
        if b.is_ascii_digit() {
            saw_digit = true;
            whole = whole.saturating_mul(10).saturating_add((b - b'0') as u32);
            i += 1;
        } else {
            break;
        }
    }
    if !saw_digit {
        return None;
    }

    let mut frac = 0u32;
    let mut scale = 1u32;
    if i < line.len() && line[i] == b'.' {
        i += 1;
        let mut frac_digits = 0u32;
        while i < line.len() && line[i].is_ascii_digit() {
            if frac_digits < 3 {
                frac = frac.saturating_mul(10).saturating_add((line[i] - b'0') as u32);
                scale = scale.saturating_mul(10);
                frac_digits += 1;
            }
            i += 1;
        }
    }

    if i != line.len() {
        return None;
    }
    Some(whole as f32 + (frac as f32 / scale as f32))
}
