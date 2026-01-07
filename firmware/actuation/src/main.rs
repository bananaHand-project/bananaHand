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
    peripherals::{ADC1, DMA1_CH1, TIM1},
    time::Hertz,
    timer::{
        GeneralInstance4Channel,
        low_level::CountingMode,
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_time::{Duration, Ticker};
use fmt::{info, unwrap};

use core::sync::atomic::{AtomicU16, AtomicU32, Ordering};

pub struct Pq12PCal;
impl Pq12PCal {
    pub const STROKE_LENGTH: f32 = 20.0;
    pub const ADC_MAX_RAW: u16 = 4095;

    pub fn raw_to_mm(raw: u16) -> f32 {
        (raw as f32) * (Self::STROKE_LENGTH / Self::ADC_MAX_RAW as f32)
    }
}

pub struct Pq12PDrive<'a, T: GeneralInstance4Channel> {
    pwm_1: SimplePwmChannel<'a, T>,
    pwm_2: SimplePwmChannel<'a, T>,
}

impl<'a, T: GeneralInstance4Channel> Pq12PDrive<'a, T> {
    /// Create a new Pq12Drive instance.
    pub fn new(
        pwm_1: SimplePwmChannel<'a, T>,
        pwm_2: SimplePwmChannel<'a, T>,
    ) -> Pq12PDrive<'a, T> {
        Pq12PDrive { pwm_1, pwm_2 }
    }

    /// Set motor to coast (PWM1 = 0 PWM2 = 0).
    pub fn coast(&mut self) {
        self.pwm_1.set_duty_cycle_fully_off();
        self.pwm_2.set_duty_cycle_fully_off();
    }

    /// Set motor to brake (PWM1 = 1 PWM2 = 1).
    pub fn brake(&mut self) {
        self.pwm_1.set_duty_cycle_fully_on();
        self.pwm_2.set_duty_cycle_fully_on();
    }

    /// Move PQ12 plunger out (PWM1 = DC% PWM2 = 0).
    pub fn move_out(&mut self, duty_cycle_percent: u8) {
        let dcp = duty_cycle_percent.clamp(0, 100);
        self.pwm_1.set_duty_cycle_fully_on();
        self.pwm_2.set_duty_cycle_percent(100 - dcp);
    }

    /// Move PQ12 plunger in (PWM1 = 0 PWM2 = DC%).
    pub fn move_in(&mut self, duty_cycle_percent: u8) {
        let dcp = duty_cycle_percent.clamp(0, 100);
        self.pwm_1.set_duty_cycle_percent(100 - dcp);

        self.pwm_2.set_duty_cycle_fully_on();
    }
}

pub const NUM_MOTORS: usize = 2;
pub static POSITION_RAW: [AtomicU16; NUM_MOTORS] = [
    AtomicU16::new(0),
    AtomicU16::new(0),
    // AtomicU16::new(0),
    // AtomicU16::new(0),
    // AtomicU16::new(0),
];

pub static POSITION_SEQ: AtomicU32 = AtomicU32::new(0);

#[embassy_executor::task]
pub async fn adc_sampler_task(
    mut adc: embassy_stm32::adc::Adc<'static, ADC1>,
    mut dma: embassy_stm32::Peri<'static, DMA1_CH1>,
    mut pos_ch: [embassy_stm32::adc::AnyAdcChannel<ADC1>; NUM_MOTORS],
) -> ! {
    let mut buf = [0u16; NUM_MOTORS];
    let mut ticker = Ticker::every(Duration::from_hz(400)); // sample faster than control
    loop {
        info!("ADC R Start");
        // Read all 5 position channels in one shot
        adc.read(
            dma.reborrow(),
            pos_ch.iter_mut().map(|ch| (ch, SampleTime::CYCLES12_5)),
            &mut buf,
        )
        .await;

        for (i, &raw) in buf.iter().enumerate() {
            POSITION_RAW[i].store(raw, Ordering::Relaxed);
        }
        POSITION_SEQ.fetch_add(1, Ordering::Release);
        info!("ADC R End");
        ticker.next().await;
    }
}

#[derive(Clone, Copy)]
pub struct MotorGoal {
    pub target_mm: f32,
    pub duty: u8,
    pub end_in_brake: bool,
}

#[embassy_executor::task]
pub async fn motor_control_task(
    mut motors: [Pq12PDrive<'static, TIM1>; NUM_MOTORS],
    // receive goals from UART tasks via a Channel (not shown)
) -> ! {
    use embassy_time::{Duration, Ticker};

    const CONTROL_HZ: u64 = 200;
    const TOL_MM: f32 = 0.1;

    let goals = [MotorGoal {
        target_mm: 10.0,
        duty: 40,
        end_in_brake: true,
    }; NUM_MOTORS];
    let mut last_seq = POSITION_SEQ.load(Ordering::Acquire);

    let mut ticker = Ticker::every(Duration::from_hz(CONTROL_HZ));
    loop {
        info!("Motor Start");
        // (1) pull any pending goal updates from a channel here

        // (2) optional: freshness check
        let seq = POSITION_SEQ.load(Ordering::Acquire);
        let fresh = seq != last_seq;
        last_seq = seq;

        for i in 0..NUM_MOTORS {
            info!("Motor {} calc start", i);
            let raw = POSITION_RAW[i].load(Ordering::Relaxed) as u16;
            let pos_mm = Pq12PCal::raw_to_mm(raw);
            let g = goals[i];

            if !fresh {
                // if ADC stalled, fail-safe (optional)
                motors[i].coast();
                continue;
            }

            if pos_mm + TOL_MM < g.target_mm {
                motors[i].move_out(g.duty);
            } else if pos_mm - TOL_MM > g.target_mm {
                motors[i].move_in(g.duty);
            } else {
                if g.end_in_brake {
                    motors[i].brake();
                } else {
                    motors[i].coast();
                }
            }
            info!("Motor {} calc end", i);
        }

        info!("Motor End");
        ticker.next().await;
    }
}

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
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    // PWM Init
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new(p.PC0, OutputType::PushPull)),
        Some(PwmPin::new(p.PC1, OutputType::PushPull)),
        Some(PwmPin::new(p.PC2, OutputType::PushPull)),
        Some(PwmPin::new(p.PC3, OutputType::PushPull)),
        Hertz(20_000),
        CountingMode::EdgeAlignedUp,
    );
    let mut pwm_channels = pwm.split();
    pwm_channels.ch1.enable();
    pwm_channels.ch2.enable();
    pwm_channels.ch3.enable();
    pwm_channels.ch4.enable();

    let mot_1 = Pq12PDrive::new(pwm_channels.ch1, pwm_channels.ch2);
    let mot_2 = Pq12PDrive::new(pwm_channels.ch3, pwm_channels.ch4);

    // ADC Init
    let dma = p.DMA1_CH1;
    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES640_5);

    // ADC Channel Init
    let mot_1_pos_adc_pin = p.PA0.degrade_adc();
    let mot_2_pos_adc_pin = p.PA1.degrade_adc();

    led.set_high();

    // Spawn tasks.
    unwrap!(_spawner.spawn(adc_sampler_task(
        adc,
        dma,
        [mot_1_pos_adc_pin, mot_2_pos_adc_pin]
    )));
    unwrap!(_spawner.spawn(motor_control_task([mot_1, mot_2])));
}
