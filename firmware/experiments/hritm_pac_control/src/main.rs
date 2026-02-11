#![no_std]
#![no_main]

mod fmt;
mod hrtim_pwm;
mod hrtim_pwm_v2;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    gpio::{Level, Output, Speed},
    rcc::{APBPrescaler, clocks},
};
use embassy_time::{Duration, Timer};
use fmt::{info, unwrap};
use hrtim_pwm_v2::{HrtimCore, HrtimPrescaler, period_reg_val};

const PWM_FREQ: f32 = 20_0000.0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSI,  // 16 Mhz
            prediv: PllPreDiv::DIV4, // /4 = 4Mhz
            mul: PllMul::MUL85,      // x85 =   340Mhz
            divp: None,
            divq: None,
            divr: Some(PllRDiv::DIV2), // /2 = 170MHz
        });
        config.rcc.mux.adc12sel = mux::Adcsel::SYS;
        config.rcc.sys = Sysclk::PLL1_R;
    }

    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let period = unwrap!(period_reg_val(
        clocks(&p.RCC),
        APBPrescaler::DIV1,
        HrtimPrescaler::DIV1,
        PWM_FREQ,
    ));
    let prescaler = HrtimPrescaler::DIV32;

    let (tim_a, tim_b, tim_c, tim_d, tim_e, tim_f) = HrtimCore::new()
        .add_tim_a_ch1_ch2(*p.PA8, *p.PA9, period, prescaler) // or with_tim_a_ch2 / with_tim_a_ch1_ch2
        .add_tim_b_ch1_ch2(*p.PA10, *p.PA11, period, prescaler)
        .add_tim_c_ch1_ch2(*p.PB12, *p.PB13, period, prescaler)
        .add_tim_d_ch1_ch2(*p.PB14, *p.PB15, period, prescaler)
        .add_tim_e_ch1_ch2(*p.PC8, *p.PC9, period, prescaler)
        .add_tim_f_ch1_ch2(*p.PC6, *p.PC7, period, prescaler)
        .split_active()
        .unwrap();

    tim_a.ch1_set_dc_percent(10);
    tim_a.ch2_set_dc_percent(20);
    tim_a.ch1_en();
    tim_a.ch2_en();

    tim_b.ch1_set_dc_percent(30);
    tim_b.ch2_set_dc_percent(40);
    tim_b.ch1_en();
    tim_b.ch2_en();

    tim_c.ch1_set_dc_percent(50);
    tim_c.ch2_set_dc_percent(60);
    tim_c.ch1_en();
    tim_c.ch2_en();

    tim_d.ch1_set_dc_percent(70);
    tim_d.ch2_set_dc_percent(80);
    tim_d.ch1_en();
    tim_d.ch2_en();

    tim_e.ch1_set_dc_percent(90);
    tim_e.ch2_set_dc_percent(99);
    tim_e.ch1_en();
    tim_e.ch2_en();

    tim_f.ch1_set_dc_percent(99);
    tim_f.ch2_set_dc_percent(99);
    tim_f.ch1_en();
    tim_f.ch2_en();

    loop {
        info!("Hello, World!");
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
