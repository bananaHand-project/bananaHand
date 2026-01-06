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
    gpio::Pin,
    pac::{HRTIM1, hrtim},
};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    pac,
};
use embassy_time::{Duration, Timer};
use fmt::info;

// Timer Index:
const TIM_A: usize = 0;

// Channel Index:
const CH1: usize = 0;
const CH2: usize = 1;

// Comparator Index:
const CMP1: usize = 0;
const CMP2: usize = 1;

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

    rcc.ahb2enr().modify(|w| w.set_gpioaen(true));

    rcc.apb2enr().modify(|w| w.set_hrtim1en(true));

    let gpioa = pac::GPIOA;

    gpioa.moder().modify(|w| {
        w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE);
        w.set_moder(9, pac::gpio::vals::Moder::ALTERNATE);
    });

    gpioa.pupdr().modify(|w| {
        w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING);
        w.set_pupdr(9, pac::gpio::vals::Pupdr::FLOATING);
    });

    gpioa.ospeedr().modify(|w| {
        w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
        w.set_ospeedr(9, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
    });

    // AFRH index 0 -> pin 8, index 1 -> pin 9. AF13 for both.
    gpioa.afr(1).modify(|w| {
        w.set_afr(0, 13);
        w.set_afr(1, 13);
    });

    let hrtim1 = pac::HRTIM1;
    // Set Timer A (idx 0)  to operate in continuous mode.
    hrtim1.tim(TIM_A).cr().modify(|w| w.set_cont(true));
    hrtim1.tim(TIM_A).cr().modify(|w| w.set_ckpsc(5)); // Clock prescaler of /32

    // The HRTIM clocked by what appears as a  5.44Ghz clock (170Mhz x 32 = 5.44Ghz).
    // In the line above we set a prescaler of /32, brining our effective HRTIM clock to 170Mhz or a period of
    // The counting period of the clock is selected by writing to a 16-bit register using the formula:
    // PER = T_count / T_hrtim
    // Example: Want a frequency of 20kHz (period of 50us)? PER = 50us / 0.005882us = 8500

    // Set Timer A (idx 0) period to 10us.
    hrtim1.tim(TIM_A).per().modify(|w| w.set_per(8500));

    // The X% duty cycle is obtained by multiplying the period by the duty cycle: PER x DC.

    // Set Timer A Compare 1 to 50% of the Timer A period (Will result in 50% DC)
    hrtim1.tim(TIM_A).cmp(CMP1).modify(|w| w.set_cmp(8500 / 2));

    // Set Timer A Compare 1 to 25% of the Timer A period (Will result in 25% DC)
    hrtim1.tim(TIM_A).cmp(CMP2).modify(|w| w.set_cmp(8500 / 4));

    hrtim1.tim(TIM_A).setr(CH1).modify(|w| w.set_per(true)); // Tim A Ch1 set on Tim A period
    hrtim1
        .tim(TIM_A)
        .rstr(CH1)
        .modify(|w| w.set_cmp(CMP1, true)); // Tim A Ch1 reset on Tim A CMP1 event

    hrtim1.tim(TIM_A).setr(CH2).modify(|w| w.set_per(true)); // Tim A Ch2 set on Tim A period

    hrtim1
        .tim(TIM_A)
        .rstr(CH2)
        .modify(|w| w.set_cmp(CMP2, true)); // Tim A Ch2 reset on Tim A CMP2 event

    hrtim1.mcr().modify(|w| w.set_mcen(true)); // Enable hrtim globally
    hrtim1.mcr().modify(|w| w.set_tcen(TIM_A, true)); // Start Tim A
    hrtim1.oenr().modify(|w| w.set_t1oen(TIM_A, true)); // Enable Tim A Ch1 output
    hrtim1.oenr().modify(|w| w.set_t2oen(TIM_A, true)); // Enable Tim A Ch2 output

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
