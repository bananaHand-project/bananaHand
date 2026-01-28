//! Expose *simple* **but unsafe** PWM interface for Hrtim
use embassy_stm32::{self, Peri, gpio, hrtim, pac, peripherals::HRTIM1};

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
#[allow(unused)]
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
            pac::RCC.ahb2enr().modify(|w| w.[<set_gpio $port:lower en>](true));

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
macro_rules! subtimer_controls_creator {
    ($port:ident) => {
        paste! {
            pub fn [<set_tim $port:lower _ch1_dc>](&self, dc: u8) -> Result<(), HrtimError> {
                let timer = self.[<tim_ $port:lower>].as_ref().ok_or(HrtimError::ChannelNotSetup)?;
                timer.set_ch1_percent_dc(dc);
                Ok(())
            }
            pub fn [<enable_tim $port:lower _ch1>](&self) -> Result<(), HrtimError> {
                let timer = self.[<tim_ $port:lower>].as_ref().ok_or(HrtimError::ChannelNotSetup)?;
                timer.ch1_enable()?;
                Ok(())
            }
            pub fn [<set_tim $port:lower _ch2_dc>](&self, dc: u8) -> Result<(), HrtimError> {
                let timer = self.[<tim_ $port:lower>].as_ref().ok_or(HrtimError::ChannelNotSetup)?;
                timer.set_ch2_percent_dc(dc);
                Ok(())
            }
            pub fn [<enable_tim $port:lower _ch2>](&self) -> Result<(), HrtimError> {
                let timer = self.[<tim_ $port:lower>].as_ref().ok_or(HrtimError::ChannelNotSetup)?;
                timer.ch2_enable()?;
                Ok(())
            }
        }
    };
}

pub(crate) trait HrtimXPwm {
    fn init(&self) -> Result<(), HrtimError>;

    fn set_ch1_percent_dc(&self, percent_dc: u8);
    fn ch1_enable(&self) -> Result<(), HrtimError>;

    fn set_ch2_percent_dc(&self, percent_dc: u8);
    fn ch2_enable(&self) -> Result<(), HrtimError>;
}

pub(crate) trait SubtimerA: HrtimXPwm {}
pub(crate) trait SubtimerB: HrtimXPwm {}
pub(crate) trait SubtimerC: HrtimXPwm {}
pub(crate) trait SubtimerD: HrtimXPwm {}
pub(crate) trait SubtimerE: HrtimXPwm {}
pub(crate) trait SubtimerF: HrtimXPwm {}

pub struct HrtimPwmManager<
    STA: SubtimerA,
    STB: SubtimerB,
    STC: SubtimerC,
    STD: SubtimerD,
    STE: SubtimerE,
    STF: SubtimerF,
> {
    tim_a: Option<STA>,
    tim_b: Option<STB>,
    tim_c: Option<STC>,
    tim_d: Option<STD>,
    tim_e: Option<STE>,
    tim_f: Option<STF>,
}

impl<STA: SubtimerA, STB: SubtimerB, STC: SubtimerC, STD: SubtimerD, STE: SubtimerE, STF: SubtimerF>
    HrtimPwmManager<STA, STB, STC, STD, STE, STF>
{
    pub fn new(
        subtimer_a: Option<STA>,
        subtimer_b: Option<STB>,
        subtimer_c: Option<STC>,
        subtimer_d: Option<STD>,
        subtimer_e: Option<STE>,
        subtimer_f: Option<STF>,
    ) -> Result<Self, HrtimError> {
        let mut manager = Self {
            tim_a: None,
            tim_b: None,
            tim_c: None,
            tim_d: None,
            tim_e: None,
            tim_f: None,
        };

        // Enable hrtim globally
        pac::RCC.apb2enr().modify(|w| w.set_hrtim1en(true));
        pac::HRTIM1.mcr().modify(|w| w.set_mcen(true));

        if let Some(sta) = subtimer_a {
            sta.init()?;
            manager.tim_a = Some(sta);
        };
        if let Some(stb) = subtimer_b {
            stb.init()?;
            manager.tim_b = Some(stb);
        };
        if let Some(stc) = subtimer_c {
            stc.init()?;
            manager.tim_c = Some(stc);
        };
        if let Some(std) = subtimer_d {
            std.init()?;
            manager.tim_d = Some(std);
        };
        if let Some(ste) = subtimer_e {
            ste.init()?;
            manager.tim_e = Some(ste);
        };
        if let Some(stf) = subtimer_f {
            stf.init()?;
            manager.tim_f = Some(stf);
        };

        Ok(manager)
    }

    subtimer_controls_creator!(a);
    subtimer_controls_creator!(b);
    subtimer_controls_creator!(c);
    subtimer_controls_creator!(d);
    subtimer_controls_creator!(e);
    subtimer_controls_creator!(f);
}

macro_rules! subtimer_interface_creator {
    ($port:ident) => {
        paste! {
            /// Struct for HRTIM sub timer.
            pub struct [<Hrtim1 $port:upper Pwm>]<'a, P1, P2>
            where
                P1: hrtim::[<Channel $port:upper Pin>]<HRTIM1>,
                P2: hrtim::[<Channel $port:upper ComplementaryPin>]<HRTIM1>,
            {
                pub ch1_pin: Option<Peri<'a, P1>>,
                pub ch2_pin: Option<Peri<'a, P2>>,
                pub period: u16,
                pub clock_prescaler: Prescaler,
            }

            impl<'a, P1: hrtim::[<Channel $port:upper Pin>]<HRTIM1>, P2: hrtim::[<Channel $port:upper ComplementaryPin>]<HRTIM1>> HrtimXPwm
                for [<Hrtim1 $port:upper Pwm>]<'a, P1, P2>
            {
                fn init(&self) -> Result<(), HrtimError> {
                    self.setup_gpio()?;
                    self.init_subtimer();
                    self.config_channels();
                    Ok(())
                }

                fn set_ch1_percent_dc(&self, percent: u8) {
                    let cmp_set = (self.period as f32 * (percent as f32 / 100.0)) as u16;
                    pac::HRTIM1
                        .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                        .cmp(HrtimXComparator::Cmp1 as usize)
                        .modify(|w| w.set_cmp(cmp_set));
                }

                fn ch1_enable(&self) -> Result<(), HrtimError> {
                    if let Some(_) = self.ch1_pin {
                        pac::HRTIM1
                            .oenr()
                            .modify(|w| w.set_t1oen(HrtimSubTimer::[<Tim $port:upper>] as usize, true)); // Enable TimX Ch1 output
                        Ok(())
                    } else {
                        Err(HrtimError::ChannelNotSetup)
                    }
                }

                fn set_ch2_percent_dc(&self, percent: u8) {
                    let cmp_set = (self.period as f32 * (percent as f32 / 100.0)) as u16;
                    pac::HRTIM1
                        .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                        .cmp(HrtimXComparator::Cmp2 as usize)
                        .modify(|w| w.set_cmp(cmp_set));
                }

                fn ch2_enable(&self) -> Result<(), HrtimError> {
                    if let Some(_) = self.ch2_pin {
                        pac::HRTIM1
                            .oenr()
                            .modify(|w| w.set_t2oen(HrtimSubTimer::[<Tim $port:upper>] as usize, true)); // Enable TimX Ch2 output
                        Ok(())
                    } else {
                        Err(HrtimError::ChannelNotSetup)
                    }
                }
            }

            impl<'a, P1: hrtim::[<Channel $port:upper Pin>]<HRTIM1>, P2: hrtim::[<Channel $port:upper ComplementaryPin>]<HRTIM1>> [<Subtimer $port:upper>]
                for [<Hrtim1 $port:upper Pwm>]<'a, P1, P2>
            {
            }

            impl<'a, P1: hrtim::[<Channel $port:upper Pin>]<HRTIM1>, P2: hrtim::[<Channel $port:upper ComplementaryPin>]<HRTIM1>>
                [<Hrtim1 $port:upper Pwm>]<'a, P1, P2>
            {
                fn setup_gpio(&self) -> Result<(), HrtimError> {
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

                fn init_subtimer(&self) {
                    // Set TimX to operate in continuous mode.
                    pac::HRTIM1
                        .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                        .cr()
                        .modify(|w| w.set_cont(true));

                    // Set TimX clock prescaler.
                    pac::HRTIM1
                        .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                        .cr()
                        .modify(|w| w.set_ckpsc(self.clock_prescaler as u8));
                    // Set TimX clock Period.
                    pac::HRTIM1
                        .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                        .per()
                        .modify(|w| w.set_per(self.period));
                    // Start TimX
                    pac::HRTIM1
                        .mcr()
                        .modify(|w| w.set_tcen(HrtimSubTimer::[<Tim $port:upper>] as usize, true));
                }

                fn config_channels(&self) {
                    // Channel 1 Setup:
                    if let Some(_) = self.ch1_pin {
                        pac::HRTIM1
                            .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                            .setr(HrtimXChannel::Ch1 as usize)
                            .modify(|w| w.set_per(true)); // TimX Ch1 set on TimX period
                        pac::HRTIM1
                            .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                            .rstr(HrtimXChannel::Ch1 as usize)
                            .modify(|w| w.set_cmp(HrtimXComparator::Cmp1 as usize, true)); // TimX Ch1 reset on TimX CMP1 event
                    }

                    // Channel 2 Setup:
                    if let Some(_) = self.ch2_pin {
                        pac::HRTIM1
                            .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                            .setr(HrtimXChannel::Ch2 as usize)
                            .modify(|w| w.set_per(true)); // TimX Ch2 set on TimX period
                        pac::HRTIM1
                            .tim(HrtimSubTimer::[<Tim $port:upper>] as usize)
                            .rstr(HrtimXChannel::Ch2 as usize)
                            .modify(|w| w.set_cmp(HrtimXComparator::Cmp2 as usize, true)); // TimX Ch2 reset on TimX CMP2 event
                    }
                }
            }
        }
    };
}

subtimer_interface_creator!(a);
subtimer_interface_creator!(b);
subtimer_interface_creator!(c);
subtimer_interface_creator!(d);
subtimer_interface_creator!(e);
subtimer_interface_creator!(f);

// TODO:
// instead of where clause use separate impl blocks for generics P1, P2, first block if only P1 one is used, second block if only p2 is used, third block if both
// the manager will also need separate impl blocks for each subtimer variant
