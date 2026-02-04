#![no_std]
#![no_main]

mod fmt;
mod hrtim_pwm;

use crate::hrtim_pwm::Hrtim1CPwm;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config, Peri,
    gpio::{Level, Output, Speed},
    pac, peripherals,
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

    // let ch1 = p.PA8;
    // let ch2 = p.PA9;
    let clock_prescaler = hrtim_pwm::Prescaler::DIV32;
    let period = 8500u16;

    // let hrtim_a_pwm = hrtim_pwm::Hrtim1APwm {
    //     ch1_pin: Some(p.PA8),
    //     ch2_pin: Some(p.PA9),
    //     period,
    //     clock_prescaler,
    // };

    let hrtim_b_pwm = hrtim_pwm::Hrtim1BPwm {
        ch1_pin: Some(p.PA10),
        ch2_pin: Some(p.PA11),
        period,
        clock_prescaler,
    };

    let hrtim_c_pwm = hrtim_pwm::Hrtim1CPwm {
        ch1_pin: Some(p.PB12),
        ch2_pin: Some(p.PB13),
        period,
        clock_prescaler,
    };

    let hrtim_d_pwm = hrtim_pwm::Hrtim1DPwm {
        ch1_pin: Some(p.PB14),
        ch2_pin: Some(p.PB15),
        period,
        clock_prescaler,
    };

    let hrtim_e_pwm = hrtim_pwm::Hrtim1EPwm {
        ch1_pin: Some(p.PC8),
        ch2_pin: Some(p.PC9),
        period,
        clock_prescaler,
    };

    let hrtim_f_pwm = hrtim_pwm::Hrtim1FPwm {
        ch1_pin: Some(p.PC6),
        ch2_pin: Some(p.PC7),
        period,
        clock_prescaler,
    };

    let manager = unwrap!(hrtim_pwm::HrtimPwmManager::new(
        None::<hrtim_pwm::Hrtim1APwm<peripherals::PA8, peripherals::PA9>>,
        Some(hrtim_b_pwm),
        Some(hrtim_c_pwm),
        Some(hrtim_d_pwm),
        Some(hrtim_e_pwm),
        // Some(hrtim_f_pwm), // Causes assertion failed: n < 5usize in set_tcen. I think this is an issue with the PAC, it should be n < 6usize see pg. 995 stm32 ref manual (RM0440).
        None::<hrtim_pwm::Hrtim1FPwm<peripherals::PC6, peripherals::PC7>>,
    ));

    // unwrap!(manager.set_tima_ch1_dc(50));
    // unwrap!(manager.set_tima_ch2_dc(25));
    // unwrap!(manager.enable_tima_ch1());
    // unwrap!(manager.enable_tima_ch2());

    unwrap!(manager.set_timb_ch1_dc(10));
    unwrap!(manager.set_timb_ch2_dc(20));
    unwrap!(manager.enable_timb_ch1());
    unwrap!(manager.enable_timb_ch2());

    unwrap!(manager.set_timc_ch1_dc(30));
    unwrap!(manager.set_timc_ch2_dc(40));
    unwrap!(manager.enable_timc_ch1());
    unwrap!(manager.enable_timc_ch2());

    unwrap!(manager.set_timd_ch1_dc(50));
    unwrap!(manager.set_timd_ch2_dc(60));
    unwrap!(manager.enable_timd_ch1());
    unwrap!(manager.enable_timd_ch2());

    unwrap!(manager.set_time_ch1_dc(70));
    unwrap!(manager.set_time_ch2_dc(80));
    unwrap!(manager.enable_time_ch1());
    unwrap!(manager.enable_time_ch2());

    // unwrap!(manager.set_timf_ch1_dc(90));
    // unwrap!(manager.set_timf_ch2_dc(100));
    // unwrap!(manager.enable_timf_ch1());
    // unwrap!(manager.enable_timf_ch2());

    loop {
        info!("Hello, World!");
        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}
