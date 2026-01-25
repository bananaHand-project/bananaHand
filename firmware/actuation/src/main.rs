#![no_std]
#![no_main]

mod fmt;
mod hrtim_pwm;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    gpio::{Level, Output, Speed},
    pac,
};
use embassy_time::{Duration, Timer};
use fmt::{info, unwrap};

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

    // let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let rcc = pac::RCC;

    rcc.apb2enr().modify(|w| w.set_hrtim1en(true));

    pac::HRTIM1.mcr().modify(|w| w.set_mcen(true)); // Enable hrtim globally

    let ch1 = p.PA8;
    let ch2 = p.PA9;
    let clock_prescaler = hrtim_pwm::Prescaler::DIV32;
    let period = 8500u16;

    let hrtim_a_pwm = unwrap!(hrtim_pwm::Hrtim1APWM::new(
        Some(ch1),
        Some(ch2),
        period,
        clock_prescaler
    ));
    hrtim_a_pwm.set_ch1_percent_dc(100);
    hrtim_a_pwm.set_ch2_percent_dc(0);
    unwrap!(hrtim_a_pwm.ch1_enable());
    unwrap!(hrtim_a_pwm.ch2_enable());
    Timer::after_secs(2).await;
    hrtim_a_pwm.set_ch1_percent_dc(0);
    hrtim_a_pwm.set_ch2_percent_dc(100);

    loop {
        info!("Hello, World!");
        hrtim_a_pwm.set_ch1_percent_dc(25);
        hrtim_a_pwm.set_ch2_percent_dc(75);
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        hrtim_a_pwm.set_ch1_percent_dc(75);
        hrtim_a_pwm.set_ch2_percent_dc(25);
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
