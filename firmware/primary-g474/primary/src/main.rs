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
    gpio::{Level, Output, Speed},
    rcc::{APBPrescaler, clocks},
    time::Hertz,
};
use embassy_time::{Duration, Timer};
use fmt::{info};
use hrtim_pwm_hal::{HrtimCore, HrtimPrescaler, period_reg_val};

const PWM_FREQ: Hertz = Hertz(20_000);

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

    let mut led1 = Output::new(p.PC13, Level::High, Speed::Low);

    let period = period_reg_val(
        clocks(&p.RCC),
        APBPrescaler::DIV1,
        HrtimPrescaler::DIV1,
        PWM_FREQ,
    )
    .unwrap();
    info!("{}", period);
    let prescaler = HrtimPrescaler::DIV32;

    let (_, _, _, _, _, tim_f) = HrtimCore::new()
        .add_tim_f_ch1_ch2(*p.PC6, *p.PC7, period, prescaler)
        .split_active()
        .unwrap();

    tim_f.ch1_set_dc_percent(50);
    tim_f.ch2_set_dc_percent(75);
    tim_f.ch1_en();
    tim_f.ch2_en();

    loop {
        info!("Hello, World!");
        led1.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led1.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
