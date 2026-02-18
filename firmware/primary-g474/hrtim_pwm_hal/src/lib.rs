//! HAL for use of STM32G474 HRTIM1 as simple PWM channels

#![no_std]

use core::{fmt::Debug, ops::Div};

use embassy_stm32::{
    self, hrtim, pac,
    peripherals::HRTIM1,
    rcc::{APBPrescaler, Clocks},
    time::Hertz,
};

use paste::paste;

#[derive(Debug, Copy, Clone)]
pub enum HrtimError {
    GpioInitFailed,
    ClocksIncorrectlyConfigured,
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
pub enum HrtimPrescaler {
    DIV1 = 0,
    DIV2 = 1,
    DIV4 = 2,
    DIV8 = 3,
    DIV16 = 4,
    DIV32 = 5,
    DIV64 = 6,
    DIV128 = 7,
}

impl HrtimPrescaler {
    /// Convert prescaler into its integer divisor for `Hertz / divisor`.
    pub const fn divisor(self) -> u32 {
        match self {
            HrtimPrescaler::DIV1 => 1,
            HrtimPrescaler::DIV2 => 2,
            HrtimPrescaler::DIV4 => 4,
            HrtimPrescaler::DIV8 => 8,
            HrtimPrescaler::DIV16 => 16,
            HrtimPrescaler::DIV32 => 32,
            HrtimPrescaler::DIV64 => 64,
            HrtimPrescaler::DIV128 => 128,
        }
    }
}

impl From<HrtimPrescaler> for u32 {
    fn from(value: HrtimPrescaler) -> Self {
        value.divisor()
    }
}

impl Div<HrtimPrescaler> for Hertz {
    type Output = Hertz;

    fn div(self, rhs: HrtimPrescaler) -> Self::Output {
        self / rhs.divisor()
    }
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
pub enum HrtimXCompare {
    Cmp1 = 0,
    Cmp2 = 1,
}

// TODO: Integrate this into new subtimer creation

/// Calculate the necessary value of subtimer period register for a given pwm frequency in hertz and clock configuration.
pub fn period_reg_val(
    clocks: &Clocks,
    apb2_prescaler: APBPrescaler,
    hrtim_prescaler: HrtimPrescaler,
    desired_pwm_hz: f32,
) -> Result<u16, HrtimError> {
    let apparent_hrtim_clk = clocks
        .hclk1
        .to_hertz()
        .ok_or(HrtimError::ClocksIncorrectlyConfigured)?
        .div(apb2_prescaler)
        .div(hrtim_prescaler);

    Ok(((1.0 / desired_pwm_hz) / (1.0 / apparent_hrtim_clk.0 as f32)) as u16)
}

pub struct NoTimer;
pub struct NoChannel;

/// Marker returned when a timer is absent but we still call `activate()`.
pub struct NoTimerActive;

/// Trait for anything that can be turned into an active, PWM-capable handle.
pub trait Activate {
    type Active;
    fn activate(self) -> Result<Self::Active, HrtimError>;
}

impl Activate for NoTimer {
    type Active = NoTimerActive;
    fn activate(self) -> Result<Self::Active, HrtimError> {
        Ok(NoTimerActive)
    }
}

#[derive(Debug)]
pub struct HrtimCore<A = NoTimer, B = NoTimer, C = NoTimer, D = NoTimer, E = NoTimer, F = NoTimer> {
    a: A,
    b: B,
    c: C,
    d: D,
    e: E,
    f: F,
}

impl HrtimCore<NoTimer, NoTimer> {
    pub fn new() -> Self {
        // Enable hrtim globally
        pac::RCC.apb2enr().modify(|w| w.set_hrtim1en(true));
        pac::HRTIM1.mcr().modify(|w| w.set_mcen(true));
        Self {
            a: NoTimer,
            b: NoTimer,
            c: NoTimer,
            d: NoTimer,
            e: NoTimer,
            f: NoTimer,
        }
    }
}

// HrtimCore General Impls

impl<A, B, C, D, E, F> HrtimCore<A, B, C, D, E, F> {
    /// Consume the `HrtimCore` and return the configured subtimers so the caller
    /// can `activate()` and use PWM (set duty, enable outputs, etc.) on configured channels.
    pub fn split(self) -> (A, B, C, D, E, F) {
        (self.a, self.b, self.c, self.d, self.e, self.f)
    }
}

impl<A: Activate, B: Activate, C: Activate, D: Activate, E: Activate, F: Activate>
    HrtimCore<A, B, C, D, E, F>
{
    /// Consume the `HrtimCore` and return PWM-capable activated subtimers for those that were configured
    /// and `NoTimerActive` for absent timers.
    pub fn split_active(
        self,
    ) -> Result<
        (
            A::Active,
            B::Active,
            C::Active,
            D::Active,
            E::Active,
            F::Active,
        ),
        HrtimError,
    > {
        Ok((
            self.a.activate()?,
            self.b.activate()?,
            self.c.activate()?,
            self.d.activate()?,
            self.e.activate()?,
            self.f.activate()?,
        ))
    }
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

macro_rules! define_subtimer {
    ($letter:ident) => {
        paste! {
            // Marker traits to know at whether a channel is setup (present)

            trait [<Ch1Marker $letter:upper>] {
            const PRESENT: bool;
            fn setup_gpio(&self) -> Result<(), HrtimError>;
            }

            trait [<Ch2Marker $letter:upper>] {
                const PRESENT: bool;
                fn setup_gpio(&self) -> Result<(), HrtimError>;
            }

            impl [<Ch1Marker $letter:upper>] for NoChannel {
                const PRESENT: bool = false;
                fn setup_gpio(&self) -> Result<(), HrtimError> {
                    Ok(())
                }
            }

            impl [<Ch2Marker $letter:upper>] for NoChannel {
                const PRESENT: bool = false;
                fn setup_gpio(&self) -> Result<(), HrtimError> {
                    Ok(())
                }
            }

            impl<P> [<Ch1Marker $letter:upper>] for P
            where
                P: hrtim::[<Channel $letter:upper Pin>]<HRTIM1>,
            {
                const PRESENT: bool = true;
                fn setup_gpio(&self) -> Result<(), HrtimError> {
                    let alt_func_num = self.af_num();
                    let pin_num = self.pin() as usize;

                    match PinPort::try_from(self.port()) {
                        Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
                        Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
                        Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
                        Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
                        Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
                        Err(v) => panic!("Invalid port value: {v}"),
                    }
                    Ok(())
                }
            }

            impl<P> [<Ch2Marker $letter:upper>] for P
            where
                P: hrtim::[<Channel $letter:upper ComplementaryPin>]<HRTIM1>,
            {
                const PRESENT: bool = true;
                fn setup_gpio(&self) -> Result<(), HrtimError> {
                    let alt_func_num = self.af_num();
                    let pin_num = self.pin() as usize;

                    match PinPort::try_from(self.port()) {
                        Ok(PinPort::A) => gpio_x_setup!(a, pin_num, alt_func_num)?,
                        Ok(PinPort::B) => gpio_x_setup!(b, pin_num, alt_func_num)?,
                        Ok(PinPort::C) => gpio_x_setup!(c, pin_num, alt_func_num)?,
                        Ok(PinPort::D) => gpio_x_setup!(d, pin_num, alt_func_num)?,
                        Ok(PinPort::E) => gpio_x_setup!(e, pin_num, alt_func_num)?,
                        Err(v) => panic!("Invalid port value: {v}"),
                    }
                    Ok(())
                }
            }

            // Subtimer Config

            #[derive(Debug)]
            pub struct [<SubTimer $letter:upper Config>]<Ch1 = NoChannel, Ch2 = NoChannel> {
                ch1: Ch1,
                ch2: Ch2,
                period: u16,
                clock_prescaler: HrtimPrescaler,
            }

            impl [<SubTimer $letter:upper Config>]<NoChannel, NoChannel> {
                pub fn new(period: u16, clock_prescaler: HrtimPrescaler) -> Self {
                    Self {
                        ch1: NoChannel,
                        ch2: NoChannel,
                        period,
                        clock_prescaler,
                    }
                }
            }

            impl<Ch2> [<SubTimer $letter:upper Config>]<NoChannel, Ch2> {
                pub fn with_ch1<P1: hrtim::[<Channel $letter:upper Pin>]<HRTIM1>>(self, ch1: P1) -> [<SubTimer $letter:upper Config>]<P1, Ch2> {
                    [<SubTimer $letter:upper Config>] {
                        ch1,
                        ch2: self.ch2,
                        period: self.period,
                        clock_prescaler: self.clock_prescaler,
                    }
                }
            }

            impl<Ch1> [<SubTimer $letter:upper Config>]<Ch1, NoChannel> {
                pub fn with_ch2<P2: hrtim::[<Channel $letter:upper ComplementaryPin>]<HRTIM1>>(
                    self,
                    ch2: P2,
                ) -> [<SubTimer $letter:upper Config>]<Ch1, P2> {
                    [<SubTimer $letter:upper Config>] {
                        ch1: self.ch1,
                        ch2,
                        period: self.period,
                        clock_prescaler: self.clock_prescaler,
                    }
                }
            }

            // Subtimer PWM Handle

            pub struct [<Subtimer $letter:upper Pwm>]<Ch1, Ch2> {
                ch1: Ch1,
                ch2: Ch2,
                period: u16,
                clock_prescaler: HrtimPrescaler,
            }

            impl<Ch1, Ch2> [<SubTimer $letter:upper Config>]<Ch1, Ch2>
            where
                Ch1: [<Ch1Marker $letter:upper>],
                Ch2: [<Ch2Marker $letter:upper>],
            {
                /// Consume the config, perform hardware configuration, and return a PWM-capable handle.
                pub fn activate(self) -> Result<[<Subtimer $letter:upper Pwm>]<Ch1, Ch2>, HrtimError> {
                    // GPIO + output compare setup only for present channels
                    if Ch1::PRESENT {
                        self.ch1.setup_gpio()?;
                        configure_channel(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch1, HrtimXCompare::Cmp1);
                    }
                    if Ch2::PRESENT {
                        self.ch2.setup_gpio()?;
                        configure_channel(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch2, HrtimXCompare::Cmp2);
                    }

                    configure_timer(HrtimSubTimer::[<Tim $letter:upper>], self.clock_prescaler, self.period);

                    Ok([<Subtimer $letter:upper Pwm>] {
                        ch1: self.ch1,
                        ch2: self.ch2,
                        period: self.period,
                        clock_prescaler: self.clock_prescaler,
                    })
                }
            }

            impl<Ch1, Ch2> [<Subtimer $letter:upper Pwm>]<Ch1, Ch2>
            where
                Ch1: hrtim::[<Channel $letter:upper Pin>]<HRTIM1>,
            {
                pub fn ch1_en(&self) {
                    enable_output(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch1);
                }

                pub fn ch1_set_dc_percent(&self, dc: u8) {
                    set_cmp_for_dc(HrtimSubTimer::[<Tim $letter:upper>], HrtimXCompare::Cmp1, self.period, dc);
                }
            }

            impl<Ch1, Ch2> [<Subtimer $letter:upper Pwm>]<Ch1, Ch2>
            where
                Ch2: hrtim::[<Channel $letter:upper ComplementaryPin>]<HRTIM1>,
            {
                pub fn ch2_en(&self) {
                    enable_output(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch2);
                }

                pub fn ch2_set_dc_percent(&self, dc: u8) {
                    set_cmp_for_dc(HrtimSubTimer::[<Tim $letter:upper>], HrtimXCompare::Cmp2, self.period, dc);
                }
            }

            impl<Ch1, Ch2> [<Subtimer $letter:upper Pwm>]<Ch1, Ch2> {
                pub fn get_period(&self) -> u16 {
                    self.period
                }

                pub fn get_clock_prescaler(&self) -> HrtimPrescaler {
                    self.clock_prescaler
                }

                pub fn set_period(&self, period: u16) {
                    configure_timer(HrtimSubTimer::[<Tim $letter:upper>], self.clock_prescaler, period);
                }

                pub fn set_clock_prescaler(&self, prescaler: HrtimPrescaler) {
                    configure_timer(HrtimSubTimer::[<Tim $letter:upper>], prescaler, self.period);
                }
            }

            // Activate Impls

            impl<Ch1, Ch2> Activate for [<SubTimer $letter:upper Config>]<Ch1, Ch2>
            where
                Ch1: [<Ch1Marker $letter:upper>],
                Ch2: [<Ch2Marker $letter:upper>],
            {
                type Active = [<Subtimer $letter:upper Pwm>]<Ch1, Ch2>;
                fn activate(self) -> Result<Self::Active, HrtimError> {
                    if Ch1::PRESENT {
                        self.ch1.setup_gpio()?;
                        configure_channel(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch1, HrtimXCompare::Cmp1);
                    }
                    if Ch2::PRESENT {
                        self.ch2.setup_gpio()?;
                        configure_channel(HrtimSubTimer::[<Tim $letter:upper>], HrtimXChannel::Ch2, HrtimXCompare::Cmp2);
                    }

                    configure_timer(HrtimSubTimer::[<Tim $letter:upper>], self.clock_prescaler, self.period);

                    Ok([<Subtimer $letter:upper Pwm>] {
                        ch1: self.ch1,
                        ch2: self.ch2,
                        period: self.period,
                        clock_prescaler: self.clock_prescaler,
                    })
                }
            }
        }
    };
}

define_subtimer!(a);
// Methods to add subtimer A
impl<B, C, D, E, F> HrtimCore<NoTimer, B, C, D, E, F> {
    pub fn add_tim_a<T>(self, timer: T) -> HrtimCore<T, B, C, D, E, F> {
        HrtimCore {
            a: timer,
            b: self.b,
            c: self.c,
            d: self.d,
            e: self.e,
            f: self.f,
        }
    }

    /// Add subtimer config with channel 1 present.
    pub fn add_tim_a_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<SubTimerAConfig<P1, NoChannel>, B, C, D, E, F>
    where
        P1: hrtim::ChannelAPin<HRTIM1>,
    {
        self.add_tim_a(SubTimerAConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    /// Add subtimer config with channel 2 present.
    pub fn add_tim_a_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<SubTimerAConfig<NoChannel, P2>, B, C, D, E, F>
    where
        P2: hrtim::ChannelAComplementaryPin<HRTIM1>,
    {
        self.add_tim_a(SubTimerAConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    /// Add subtimer config with channel 1 and channel 2 present.
    pub fn add_tim_a_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<SubTimerAConfig<P1, P2>, B, C, D, E, F>
    where
        P1: hrtim::ChannelAPin<HRTIM1>,
        P2: hrtim::ChannelAComplementaryPin<HRTIM1>,
    {
        self.add_tim_a(
            SubTimerAConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

define_subtimer!(b);
// Methods to add subtimer B
impl<A, C, D, E, F> HrtimCore<A, NoTimer, C, D, E, F> {
    pub fn add_tim_b<T>(self, timer: T) -> HrtimCore<A, T, C, D, E, F> {
        HrtimCore {
            a: self.a,
            b: timer,
            c: self.c,
            d: self.d,
            e: self.e,
            f: self.f,
        }
    }

    /// Add subtimer config with channel 1 present.
    pub fn add_tim_b_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, SubTimerBConfig<P1, NoChannel>, C, D, E, F>
    where
        P1: hrtim::ChannelBPin<HRTIM1>,
    {
        self.add_tim_b(SubTimerBConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    /// Add subtimer config with channel 2 present.
    pub fn add_tim_b_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, SubTimerBConfig<NoChannel, P2>, C, D, E, F>
    where
        P2: hrtim::ChannelBComplementaryPin<HRTIM1>,
    {
        self.add_tim_b(SubTimerBConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    /// Add subtimer config with channel 1 and channel 2 present.
    pub fn add_tim_b_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, SubTimerBConfig<P1, P2>, C, D, E, F>
    where
        P1: hrtim::ChannelBPin<HRTIM1>,
        P2: hrtim::ChannelBComplementaryPin<HRTIM1>,
    {
        self.add_tim_b(
            SubTimerBConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

define_subtimer!(c);
// Methods to add subtimer C
impl<A, B, D, E, F> HrtimCore<A, B, NoTimer, D, E, F> {
    pub fn add_tim_c<T>(self, timer: T) -> HrtimCore<A, B, T, D, E, F> {
        HrtimCore {
            a: self.a,
            b: self.b,
            c: timer,
            d: self.d,
            e: self.e,
            f: self.f,
        }
    }

    pub fn add_tim_c_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, SubTimerCConfig<P1, NoChannel>, D, E, F>
    where
        P1: hrtim::ChannelCPin<HRTIM1>,
    {
        self.add_tim_c(SubTimerCConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    pub fn add_tim_c_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, SubTimerCConfig<NoChannel, P2>, D, E, F>
    where
        P2: hrtim::ChannelCComplementaryPin<HRTIM1>,
    {
        self.add_tim_c(SubTimerCConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    pub fn add_tim_c_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, SubTimerCConfig<P1, P2>, D, E, F>
    where
        P1: hrtim::ChannelCPin<HRTIM1>,
        P2: hrtim::ChannelCComplementaryPin<HRTIM1>,
    {
        self.add_tim_c(
            SubTimerCConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

define_subtimer!(d);
// Methods to add subtimer D
impl<A, B, C, E, F> HrtimCore<A, B, C, NoTimer, E, F> {
    pub fn add_tim_d<T>(self, timer: T) -> HrtimCore<A, B, C, T, E, F> {
        HrtimCore {
            a: self.a,
            b: self.b,
            c: self.c,
            d: timer,
            e: self.e,
            f: self.f,
        }
    }

    pub fn add_tim_d_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, SubTimerDConfig<P1, NoChannel>, E, F>
    where
        P1: hrtim::ChannelDPin<HRTIM1>,
    {
        self.add_tim_d(SubTimerDConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    pub fn add_tim_d_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, SubTimerDConfig<NoChannel, P2>, E, F>
    where
        P2: hrtim::ChannelDComplementaryPin<HRTIM1>,
    {
        self.add_tim_d(SubTimerDConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    pub fn add_tim_d_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, SubTimerDConfig<P1, P2>, E, F>
    where
        P1: hrtim::ChannelDPin<HRTIM1>,
        P2: hrtim::ChannelDComplementaryPin<HRTIM1>,
    {
        self.add_tim_d(
            SubTimerDConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

define_subtimer!(e);
// Methods to add subtimer E
impl<A, B, C, D, F> HrtimCore<A, B, C, D, NoTimer, F> {
    pub fn add_tim_e<T>(self, timer: T) -> HrtimCore<A, B, C, D, T, F> {
        HrtimCore {
            a: self.a,
            b: self.b,
            c: self.c,
            d: self.d,
            e: timer,
            f: self.f,
        }
    }

    pub fn add_tim_e_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, SubTimerEConfig<P1, NoChannel>, F>
    where
        P1: hrtim::ChannelEPin<HRTIM1>,
    {
        self.add_tim_e(SubTimerEConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    pub fn add_tim_e_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, SubTimerEConfig<NoChannel, P2>, F>
    where
        P2: hrtim::ChannelEComplementaryPin<HRTIM1>,
    {
        self.add_tim_e(SubTimerEConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    pub fn add_tim_e_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, SubTimerEConfig<P1, P2>, F>
    where
        P1: hrtim::ChannelEPin<HRTIM1>,
        P2: hrtim::ChannelEComplementaryPin<HRTIM1>,
    {
        self.add_tim_e(
            SubTimerEConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

define_subtimer!(f);
// Methods to add subtimer F
impl<A, B, C, D, E> HrtimCore<A, B, C, D, E, NoTimer> {
    pub fn add_tim_f<T>(self, timer: T) -> HrtimCore<A, B, C, D, E, T> {
        HrtimCore {
            a: self.a,
            b: self.b,
            c: self.c,
            d: self.d,
            e: self.e,
            f: timer,
        }
    }

    pub fn add_tim_f_ch1<P1>(
        self,
        ch1: P1,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, E, SubTimerFConfig<P1, NoChannel>>
    where
        P1: hrtim::ChannelFPin<HRTIM1>,
    {
        self.add_tim_f(SubTimerFConfig::new(period, clock_prescaler).with_ch1(ch1))
    }

    pub fn add_tim_f_ch2<P2>(
        self,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, E, SubTimerFConfig<NoChannel, P2>>
    where
        P2: hrtim::ChannelFComplementaryPin<HRTIM1>,
    {
        self.add_tim_f(SubTimerFConfig::new(period, clock_prescaler).with_ch2(ch2))
    }

    pub fn add_tim_f_ch1_ch2<P1, P2>(
        self,
        ch1: P1,
        ch2: P2,
        period: u16,
        clock_prescaler: HrtimPrescaler,
    ) -> HrtimCore<A, B, C, D, E, SubTimerFConfig<P1, P2>>
    where
        P1: hrtim::ChannelFPin<HRTIM1>,
        P2: hrtim::ChannelFComplementaryPin<HRTIM1>,
    {
        self.add_tim_f(
            SubTimerFConfig::new(period, clock_prescaler)
                .with_ch1(ch1)
                .with_ch2(ch2),
        )
    }
}

// PAC helpers used above
fn configure_timer(timer: HrtimSubTimer, prescaler: HrtimPrescaler, period: u16) {
    pac::HRTIM1
        .tim(timer as usize)
        .cr()
        .modify(|w| w.set_cont(true));

    pac::HRTIM1
        .tim(timer as usize)
        .cr()
        .modify(|w| w.set_ckpsc(prescaler as u8));

    pac::HRTIM1
        .tim(timer as usize)
        .per()
        .modify(|w| w.set_per(period));

    pac::HRTIM1
        .mcr()
        .modify(|w| w.set_tcen(timer as usize, true));
}

fn configure_channel(timer: HrtimSubTimer, channel: HrtimXChannel, comparator: HrtimXCompare) {
    pac::HRTIM1
        .tim(timer as usize)
        .setr(channel as usize)
        .modify(|w| w.set_per(true));

    pac::HRTIM1
        .tim(timer as usize)
        .rstr(channel as usize)
        .modify(|w| w.set_cmp(comparator as usize, true));
}

fn enable_output(timer: HrtimSubTimer, channel: HrtimXChannel) {
    match channel {
        HrtimXChannel::Ch1 => pac::HRTIM1
            .oenr()
            .modify(|w| w.set_t1oen(timer as usize, true)),
        HrtimXChannel::Ch2 => pac::HRTIM1
            .oenr()
            .modify(|w| w.set_t2oen(timer as usize, true)),
    }
}

fn set_cmp_for_dc(timer: HrtimSubTimer, cmp: HrtimXCompare, period: u16, percent: u8) {
    let cmp_set = (period as f32 * (percent as f32 / 100.0)) as u16;
    pac::HRTIM1
        .tim(timer as usize)
        .cmp(cmp as usize)
        .modify(|w| w.set_cmp(cmp_set));
}
