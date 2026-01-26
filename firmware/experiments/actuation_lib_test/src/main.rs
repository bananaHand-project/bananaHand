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
};
use embassy_time::Timer;
use fmt::info;
use pq12p_actuator::Pq12P;

// Simple firmware target to exercise the pq12p_actuator driver on a NUCLEO-G474RE.
//
// Pin map (change if your wiring differs):
// - PWM out: TIM1_CH1 on PA6 -> DRV8876 IN1
// - PWM out: TIM1_CH2 on PC7 -> DRV8876 IN2
// - Position feedback: ADC1_IN1 on PA0 -> PQ12 potentiometer
// - Current sense: ADC1_IN2 on PA1 -> DRV8876 IPROPI (through R_IPROPI)
// - Status LED: PA5 (on-board LED)
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
    info!("Peripherals initialized.");

    let embassy_stm32::Peripherals {
        TIM3,
        ADC1,
        PA0,
        PA1,
        ..
    } = p;

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let pwm = SimplePwm::new(
        TIM3,
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

    let mut dma = p.DMA1_CH1;
    let mut adc = Adc::new(ADC1);
    adc.set_sample_time(SampleTime::CYCLES640_5);

    let mut actuator: Pq12P<'_, embassy_stm32::peripherals::TIM3, ADC1> = Pq12P::new(
        pwm_channels.ch1,
        pwm_channels.ch2,
        PA0.degrade_adc(),
        PA1.degrade_adc(),
    );

    info!("Starting PQ12 test");
    led.set_high();

    let current_i_raw = actuator.read_current_raw_async(&mut adc, &mut dma).await;
    info!("Current I reading (RAW): {}", current_i_raw);
    let current_i_ma = actuator.read_current_mA_async(&mut adc, &mut dma).await;
    info!("Current I reading (mA): {}", current_i_ma);
    let current_pos_raw = actuator.read_position_raw_async(&mut adc, &mut dma).await;
    info!("Current position reading (RAW): {}", current_pos_raw);
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Current position reading (mm): {}", current_pos_mm);

    Timer::after_secs(3).await;

    info!("Retract actuator fully");
    actuator
        .move_to_position(5.0, 90, true, &mut adc, &mut dma)
        .await;
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Homing complete");
    info!("Current position reading (mm): {}", current_pos_mm);
    Timer::after_secs(3).await;
    actuator
        .move_to_position(15.0, 90, true, &mut adc, &mut dma)
        .await;
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Move complete");
    info!("Current position reading (mm): {}", current_pos_mm);
    Timer::after_secs(3).await;

    info!("Returning to 0.0 mm");
    actuator
        .move_to_position(0.0, 20, true, &mut adc, &mut dma)
        .await;
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Move complete");
    info!("Current position reading (mm): {}", current_pos_mm);
    Timer::after_secs(3).await;

    info!("Move to 19.0 mm, end in coast");
    actuator
        .move_to_position(19.0, 20, false, &mut adc, &mut dma)
        .await;
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Move complete");
    info!("Current position reading (mm): {}", current_pos_mm);
    info!("Backdrive plunger [8 second window]");
    Timer::after_secs(8).await;

    info!("Returning to 0.0 mm");
    actuator
        .move_to_position(0.0, 90, false, &mut adc, &mut dma)
        .await;
    let current_pos_mm = actuator.read_position_mm_async(&mut adc, &mut dma).await;
    info!("Move complete");
    info!("Current position reading (mm): {}", current_pos_mm);

    actuator.coast();
    led.set_low();

    loop {
        info!("Tests complete");
        led.toggle();
        Timer::after_secs(1).await;
    }
}
