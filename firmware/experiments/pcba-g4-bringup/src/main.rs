#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use core::{default, str};

use embassy_executor::{Spawner, task};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{Config as UartConfig, UartRx};
use embassy_stm32::{
    adc::{AdcChannel, AnyAdcChannel},
    gpio::{Level, Output, Speed},
    peripherals::{ADC1, ADC2},
};
use embassy_stm32::{bind_interrupts, gpio::OutputType, time::khz, timer::simple_pwm};
use embassy_time::{Duration, Timer};
use fmt::{Bytes, info};
use pq12p_actuator;

bind_interrupts!(struct Irqs {
    USART2 => embassy_stm32::usart::InterruptHandler<embassy_stm32::peripherals::USART2>;
});

#[task]
async fn uart_reader_task(mut rx: UartRx<'static, Async>) {
    let mut rx_buf = [0u8; 64];
    let mut line = [0u8; 64];
    let mut line_len = 0usize;

    loop {
        let n = match rx.read_until_idle(&mut rx_buf).await {
            Ok(n) => n,
            Err(_) => continue,
        };

        for &b in &rx_buf[..n] {
            if b == b'\n' {
                let msg = if line_len > 0 && line[line_len - 1] == b'\r' {
                    &line[..line_len - 1]
                } else {
                    &line[..line_len]
                };

                if let Ok(text) = str::from_utf8(msg) {
                    info!("G4 received: {}", text);
                } else {
                    info!("G4 received (bytes): {}", Bytes(msg));
                }

                line_len = 0;
                continue;
            }

            if line_len < line.len() {
                line[line_len] = b;
                line_len += 1;
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led1 = Output::new(p.PC13, Level::High, Speed::Low);
    let mut led2 = Output::new(p.PC14, Level::High, Speed::Low);

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;
    let uart_rx = UartRx::new(p.USART2, Irqs, p.PA15, p.DMA1_CH5, uart_config).unwrap();
    spawner.spawn(uart_reader_task(uart_rx)).unwrap();

    let ch3 = simple_pwm::PwmPin::new(p.PC8, OutputType::PushPull);
    let ch4 = simple_pwm::PwmPin::new(p.PC9, OutputType::PushPull);
    let mut pwm = simple_pwm::SimplePwm::new(
        p.TIM8,
        None,
        None,
        Some(ch3),
        Some(ch4),
        khz(20),
        Default::default(),
    );

    let mut pwm_channels = pwm.split();
    pwm_channels.ch3.enable();
    pwm_channels.ch4.enable();

    // pwm_channels.ch3.set_duty_cycle_fully_on();
    // pwm_channels.ch4.set_duty_cycle_percent(50);

    let pos_adc: AnyAdcChannel<'_, ADC1> = p.PB1.degrade_adc();
    let i_adc: AnyAdcChannel<'_, ADC2> = p.PC5.degrade_adc();

    let mut actuator =
        pq12p_actuator::Pq12P::new(pwm_channels.ch4, pwm_channels.ch3, pos_adc, i_adc);

    actuator.move_in(50);

    loop {
        led1.set_high();
        led2.set_low();
        Timer::after(Duration::from_millis(500)).await;
        led1.set_low();
        led2.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }
}
