#![no_std]
#![no_main]

mod fmt;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::pac::{HRTIM1, hrtim};
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    pac,
};
use embassy_time::{Duration, Timer};
use fmt::info;

const TIM_A: usize = 0;
const CH1: usize = 0;
// const CH2: usize = 1;
const CMP1: usize = 0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let rcc = pac::RCC;

    rcc.ahb2enr().modify(|w| w.set_gpioaen(true));

    rcc.apb2enr().modify(|w| w.set_hrtim1en(true));

    // rcc.apb2rstr().modify(|w| {
    //     w.set_hrtim1rst(true);
    //     w.set_hrtim1rst(false);
    // });

    let gpioa = pac::GPIOA;

    gpioa.moder().modify(|w| {
        w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE);
        // w.set_moder(9, pac::gpio::vals::Moder::ALTERNATE);
    });

    gpioa.pupdr().modify(|w| {
        w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING);
        // w.set_pupdr(9, pac::gpio::vals::Pupdr::FLOATING);
    });

    gpioa.ospeedr().modify(|w| {
        w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
        // w.set_ospeedr(9, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED);
    });

    // AFRH index 0 -> pin 8, index 1 -> pin 9. AF13 for both.
    gpioa.afr(1).modify(|w| {
        w.set_afr(0, 13);
        // w.set_afr(1, 13);
    });

    let hrtim1 = pac::HRTIM1;
    // Set Timer A (idx 0)  to operate in continuous mode.
    hrtim1.tim(TIM_A).cr().modify(|w| w.set_cont(true));

    // Set Timer A (idx 0) period to __.
    hrtim1.tim(TIM_A).per().modify(|w| w.set_per(0xB400));

    // Set Timer A (idx 0) period to __.
    hrtim1.tim(TIM_A).cmp(CMP1).modify(|w| w.set_cmp(0x5A00));

    hrtim1.tim(TIM_A).setr(CH1).modify(|w| w.set_per(true)); // Tim A Ch1 set on Tim A period
    hrtim1
        .tim(TIM_A)
        .rstr(CH1)
        .modify(|w| w.set_cmp(CMP1, true)); // Tim A Ch1 reset on Tim A CMP1 event

    hrtim1.mcr().modify(|w| w.set_mcen(true)); // Enable hrtim globally
    hrtim1.mcr().modify(|w| w.set_tcen(TIM_A, true)); // Start Tim A
    hrtim1.oenr().modify(|w| w.set_t1oen(TIM_A, true)); // Enable Tim A output

    loop {
        info!("Hello, World!");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
