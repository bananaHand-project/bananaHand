//! Expose *simple* **but unsafe** PWM interface for Hrtim

use embassy_stm32::{
    self, Peri,
    gpio::AnyPin,
    hrtim::{self},
    pac::{self, RCC},
    peripherals::HRTIM1,
};

use paste::paste;

#[cfg_attr(feature = "defmt", derive(::defmt::Format))]
#[derive(Debug, Copy, Clone)]
pub enum HrtimError {
    GpioInitFailed,
    ChannelNotSetup,
}

/// HRTIM Subtimer.
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimSubTimer {
    TimA = 0,
    TimB = 1,
    TimC = 2,
    TimD = 3,
    TimE = 4,
    TimF = 5,
}

/// Prescaler applied from HRTIM clock frequency to sub timer.
#[repr(u8)]
#[derive(Debug, Copy, Clone)]
pub enum Prescaler {
    DIV1 = 0,
    DIV2 = 1,
    DIV4 = 2,
    DIV8 = 3,
    DIV16 = 4,
    DIV32 = 5,
    DIV64 = 6,
    DIV128 = 7,
}

/// Letter group of pin (e.g. A, B, C, ...)
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum PinPort {
    A = 0,
    B = 1,
    C = 2,
    D = 3,
    E = 4,
}

impl TryFrom<u8> for PinPort {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(PinPort::A),
            1 => Ok(PinPort::B),
            2 => Ok(PinPort::C),
            3 => Ok(PinPort::D),
            4 => Ok(PinPort::E),
            other => Err(other),
        }
    }
}

/// Subtimer X channel
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimXChannel {
    Ch1 = 0,
    Ch2 = 1,
}

/// Subtimer X comparator (a.k.a crossbar)
#[repr(usize)]
#[derive(Debug, Copy, Clone)]
pub enum HrtimXComparator {
    Cmp1 = 0,
    Cmp2 = 1,
}

macro_rules! gpio_x_setup {
    ($port:ident, $pin:expr, $af:expr) => {{
        paste! {
            // Enable GPIO clock: set_gpioaen / set_gpioben / ...
            RCC.ahb2enr().modify(|w| w.[<set_gpio $port:lower en>](true));

            // Pick GPIOA / GPIOB / ...
            let gpio = pac::[<GPIO $port:upper>];

            gpio.moder().modify(|w| w.set_moder($pin, pac::gpio::vals::Moder::ALTERNATE));
            gpio.pupdr().modify(|w| w.set_pupdr($pin, pac::gpio::vals::Pupdr::FLOATING));
            gpio.ospeedr().modify(|w| w.set_ospeedr($pin, pac::gpio::vals::Ospeedr::VERY_HIGH_SPEED));

            // Set appropriate alternate function. AFRL: pins 0..=7, AFRH: pins 8..=15
            match $pin {
                0..=7 => { gpio.afr(0).modify(|w| w.set_afr($pin, $af)); Ok(()) }
                8..=15 => { gpio.afr(1).modify(|w| w.set_afr($pin - 8, $af)); Ok(()) }
                _ => Err(HrtimError::GpioInitFailed),
            }
        }
    }};
}

// TODO: Make a HRTIM Pwm manger for controlling all subtimers, and controlling global intializations for HRTIM.
// TODO: Make MACRO to create structs for all other subtimers.s
pub struct Hrtim1APWM<
    'a,
    P1: hrtim::ChannelAPin<HRTIM1>,
    P2: hrtim::ChannelAComplementaryPin<HRTIM1>,
> {
    ch1_pin: Option<Peri<'a, P1>>,
    ch2_pin: Option<Peri<'a, P2>>,
    period: u16,
    clock_prescaler: Prescaler,
}

impl<'a, P1: hrtim::ChannelAPin<HRTIM1>, P2: hrtim::ChannelAComplementaryPin<HRTIM1>>
    Hrtim1APWM<'a, P1, P2>
{
    pub fn new(
        ch1_pin: Option<Peri<'a, P1>>,
        ch2_pin: Option<Peri<'a, P2>>,
        period: u16,
        clock_prescaler: Prescaler,
    ) -> Result<Hrtim1APWM<'a, P1, P2>, HrtimError> {
        let mut hrtim1_a_pwm = Hrtim1APWM {
            ch1_pin,
            ch2_pin,
            period,
            clock_prescaler,
        };

        hrtim1_a_pwm.setup_gpio()?;
        hrtim1_a_pwm.init_subtimer(hrtim1_a_pwm.clock_prescaler, hrtim1_a_pwm.period);
        hrtim1_a_pwm.config_channels();

        Ok(hrtim1_a_pwm)
    }

    pub fn setup_gpio(&self) -> Result<(), HrtimError> {
        if let Some(pin) = &self.ch1_pin {
            let alt_func_num = pin.af_num();
            let pin_num = pin.pin() as usize;

            match PinPort::try_from(pin.port()) {
                Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
                Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
                Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
                Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
                Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
                Err(v) => panic!("Invalid port value: {v}"),
            }
        }
        if let Some(pin) = &self.ch2_pin {
            let alt_func_num = pin.af_num();
            let pin_num = pin.pin() as usize;

            match PinPort::try_from(pin.port()) {
                Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
                Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
                Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
                Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
                Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
                Err(v) => panic!("Invalid port value: {v}"),
            }
        }
        Ok(())
    }

    fn init_subtimer(&mut self, clock_prescaler: Prescaler, period: u16) {
        // Set Timer A to operate in continuous mode.
        pac::HRTIM1
            .tim(HrtimSubTimer::TimA as usize)
            .cr()
            .modify(|w| w.set_cont(true));

        // Set Timer A clock prescaler.
        pac::HRTIM1
            .tim(HrtimSubTimer::TimA as usize)
            .cr()
            .modify(|w| w.set_ckpsc(clock_prescaler as u8));
        // Set Timer A clock Period.
        pac::HRTIM1
            .tim(HrtimSubTimer::TimA as usize)
            .per()
            .modify(|w| w.set_per(self.period));
        // Start Timer A
        pac::HRTIM1
            .mcr()
            .modify(|w| w.set_tcen(HrtimSubTimer::TimA as usize, true));
    }

    fn config_channels(&self) {
        // Channel 1 Setup:
        if let Some(_) = self.ch1_pin {
            pac::HRTIM1
                .tim(HrtimSubTimer::TimA as usize)
                .setr(HrtimXChannel::Ch1 as usize)
                .modify(|w| w.set_per(true)); // Tim A Ch1 set on Tim A period
            pac::HRTIM1
                .tim(HrtimSubTimer::TimA as usize)
                .rstr(HrtimXChannel::Ch1 as usize)
                .modify(|w| w.set_cmp(HrtimXComparator::Cmp1 as usize, true)); // Tim A Ch1 reset on Tim A CMP1 event
        }

        // Channel 2 Setup:
        if let Some(_) = self.ch2_pin {
            pac::HRTIM1
                .tim(HrtimSubTimer::TimA as usize)
                .setr(HrtimXChannel::Ch2 as usize)
                .modify(|w| w.set_per(true)); // Tim A Ch2 set on Tim A period
            pac::HRTIM1
                .tim(HrtimSubTimer::TimA as usize)
                .rstr(HrtimXChannel::Ch2 as usize)
                .modify(|w| w.set_cmp(HrtimXComparator::Cmp2 as usize, true)); // Tim A Ch2 reset on Tim A CMP2 event
        }
    }

    pub fn set_ch1_percent_dc(&self, percent: u8) {
        let cmp_set = (self.period as f32 * (percent as f32 / 100.0)) as u16;
        // let cmp_set = self.period / (100 - percent as u16);
        pac::HRTIM1
            .tim(HrtimSubTimer::TimA as usize)
            .cmp(HrtimXComparator::Cmp1 as usize)
            .modify(|w| w.set_cmp(cmp_set));
    }

    pub fn set_ch2_percent_dc(&self, percent: u8) {
        let cmp_set = (self.period as f32 * (percent as f32 / 100.0)) as u16;
        // let cmp_set = self.period / (100 - percent as u16);
        pac::HRTIM1
            .tim(HrtimSubTimer::TimA as usize)
            .cmp(HrtimXComparator::Cmp2 as usize)
            .modify(|w| w.set_cmp(cmp_set));
    }

    pub fn ch1_enable(&self) -> Result<(), HrtimError> {
        if let Some(_) = self.ch1_pin {
            pac::HRTIM1
                .oenr()
                .modify(|w| w.set_t1oen(HrtimSubTimer::TimA as usize, true)); // Enable Tim A Ch2 output
            Ok(())
        } else {
            Err(HrtimError::ChannelNotSetup)
        }
    }

    pub fn ch2_enable(&self) -> Result<(), HrtimError> {
        if let Some(_) = self.ch2_pin {
            pac::HRTIM1
                .oenr()
                .modify(|w| w.set_t2oen(HrtimSubTimer::TimA as usize, true)); // Enable Tim A Ch2 output
            Ok(())
        } else {
            Err(HrtimError::ChannelNotSetup)
        }
    }
}
