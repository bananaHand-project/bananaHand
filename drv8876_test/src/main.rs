#![no_std]
#![no_main]

mod fmt;

use core::time::Duration;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_stm32::{
    Config,
    adc::{Adc, AdcChannel, AnyAdcChannel, Instance, SampleTime},
    can::filter::StandardFilter,
    gpio::{Level, Output, OutputType, Speed},
    peripherals::{ADC1, TIM3},
    time::khz,
    timer::{
        Ch2, GeneralInstance4Channel,
        simple_pwm::{PwmPin, SimplePwm, SimplePwmChannel},
    },
};
use embassy_time::{Instant, Timer};
use fmt::info;

struct Actuator<'a, T: GeneralInstance4Channel, C: Instance> {
    pub vref_position_upper: f32,
    pub vref_position_lower: f32,
    pub pwm: SimplePwmChannel<'a, T>,
    pub dir_select: Output<'a>,
    pub adc_pin: AnyAdcChannel<C>,
}

impl<'a, T: GeneralInstance4Channel, C: Instance> Actuator<'a, T, C> {
    const STROKE_LENGTH: f32 = 20.0;
    const ADC_VREF: f32 = 3.3;
    const ADC_MAX_RAW: u16 = 4096;

    pub fn new(
        vref_position_upper: f32,
        vref_position_lower: f32,
        pwm: SimplePwmChannel<'a, T>,
        dir_select: Output<'a>,
        adc_pin: AnyAdcChannel<C>,
    ) -> Self {
        Actuator {
            vref_position_upper,
            vref_position_lower,
            pwm,
            dir_select,
            adc_pin,
        }
    }

    pub fn set_direction_in(&mut self) {
        self.dir_select.set_low();
    }

    pub fn set_direction_out(&mut self) {
        self.dir_select.set_high();
    }

    pub fn toggle_direction(&mut self) {
        self.dir_select.toggle();
    }

    pub fn set_speed(&mut self, percent: u8) {
        self.pwm.set_duty_cycle_percent(percent);
    }

    pub fn read_position_raw(&mut self, adc: &mut Adc<'_, C>) -> u16 {
        adc.blocking_read(&mut self.adc_pin)
    }

    pub fn read_position_v(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw_reading = adc.blocking_read(&mut self.adc_pin);
        let pos_as_v = (raw_reading as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF;
        pos_as_v
    }

    pub fn read_position_mm(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let pos_as_v = self.read_position_v(adc);
        let pos_in_mm = Self::STROKE_LENGTH * (pos_as_v - self.vref_position_lower)
            / (self.vref_position_upper - self.vref_position_lower);
        pos_in_mm
    }

    /// Bang-Bang Controller
    pub fn move_to_pos(
        &mut self,
        target_position_mm: f32,
        duty_cycle_percent: u8,
        adc: &mut Adc<'_, C>,
    ) {
        const TOLERANCE_MM: f32 = 0.1;

        let target = target_position_mm.clamp(1.0, 19.0);
        let mut current = self.read_position_mm(adc);

        info!("Target Pos: {}, Current Pos: {}", target, current);

        let direction = if current + TOLERANCE_MM < target {
            self.set_direction_out();
            Some(1.0_f32)
        } else if current - TOLERANCE_MM > target {
            self.set_direction_in();
            Some(-1.0_f32)
        } else {
            None
        };

        if direction.is_none() {
            self.set_speed(0);
            return;
        }

        self.set_speed(duty_cycle_percent);

        loop {
            current = self.read_position_mm(adc);

            if (current - target).abs() <= TOLERANCE_MM {
                info!("{}", current);
                info!("Reached target window.");
                break;
            }
        }
        self.set_speed(0);
    }

    /// P Controller
    pub fn move_to_pos_P(
        &mut self,
        kp: f32,
        target_position_mm: f32,
        // duty_cycle_percent: u8,
        adc: &mut Adc<'_, C>,
    ) {
        const TOLERANCE_MM: f32 = 0.1;

        let target = target_position_mm.clamp(1.0, 19.0);
        let mut loop_count = 0;
        loop {
            let current = self.read_position_mm(adc);

            let error = target_position_mm - current;

            let duty_cycle_percent: i8 = (error * kp) as i8;

            let duty_cycle_percent = duty_cycle_percent.clamp(-100, 100);

            if duty_cycle_percent < 0 {
                self.set_direction_in();
            } else {
                self.set_direction_out();
            }

            self.set_speed(duty_cycle_percent.abs() as u8);

            if loop_count % 1000 == 0 {
                info!(
                    "Target Pos: {}, Current Pos: {}, Error: {}, DC: {}",
                    target, current, error, duty_cycle_percent
                );
            }

            loop_count += 1;

            if (current - target).abs() <= TOLERANCE_MM {
                info!("{}", current);
                info!("Reached target window.");
                break;
            }
        }

        // let direction = if current + TOLERANCE_MM < target {
        //     self.set_direction_out();
        //     Some(1.0_f32)
        // } else if current - TOLERANCE_MM > target {
        //     self.set_direction_in();
        //     Some(-1.0_f32)
        // } else {
        //     None
        // };

        // if direction.is_none() {
        //     self.set_speed(0);
        //     return;
        // }

        // self.set_speed(duty_cycle_percent);

        // loop {
        //     current = self.read_position_mm(adc);

        //     if (current - target).abs() <= TOLERANCE_MM {
        //         info!("{}", current);
        //         info!("Reached target window.");
        //         break;
        //     }
        // }
        // self.set_speed(0);
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
    info!("Peripherals initialized.");

    let mut adc = Adc::new(p.ADC1);
    adc.set_sample_time(SampleTime::CYCLES247_5);
    info!("ADC initialized.");

    let motor_dir_pin: Output<'_> = Output::new(p.PA9, Level::High, Speed::High); // D8 on nucleo
    info!("Motor direction pin initialized.");

    let motor_pwm_pin: PwmPin<'_, TIM3, Ch2> = PwmPin::new(p.PC7, OutputType::PushPull); // PWM/D9 On nucleo
    let mut pwm = SimplePwm::new(
        p.TIM3,
        None,
        Some(motor_pwm_pin),
        None,
        None,
        khz(10),
        Default::default(),
    );
    let mut ch2 = pwm.ch2();
    ch2.enable();
    info!("PWM initialized.");
    info!("PWM max duty {}", ch2.max_duty_cycle());

    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    let mut actuator: Actuator<'_, TIM3, ADC1> =
        Actuator::new(3.3, 0.0, ch2, motor_dir_pin, p.PA0.degrade_adc());

    info!("Entering test loop.");
    let dcp = 100;
    let kp = 35.0;

    actuator.move_to_pos(1.0, dcp, &mut adc);
    // led.toggle();
    Timer::after_secs(3).await;

    // starting pwm speed test
    let start_forward = Instant::now();
    actuator.move_to_pos(19.0, dcp, &mut adc);
    let time_passed_forward = start_forward.elapsed().as_millis();
    info!("{}", time_passed_forward);
    // led.toggle();
    Timer::after_secs(3).await;

    let start_backward = Instant::now();
    actuator.move_to_pos(1.0, dcp, &mut adc);
    let time_passed_backward = start_backward.elapsed().as_millis();
    info!("{}", time_passed_backward);

    // loop {
        // actuator.move_to_pos_P(kp, 1.0, &mut adc);
        // led.toggle();
        // Timer::after_secs(2).await;
        // actuator.move_to_pos_P(kp, 19.0, &mut adc);
        // led.toggle();
        // Timer::after_secs(2).await;

        // actuator.move_to_pos(1.0, dcp, &mut adc);
        // led.toggle();
        // Timer::after_secs(2).await;
        // actuator.move_to_pos(19.5, dcp, &mut adc);
        // led.toggle();
        // Timer::after_secs(2).await;
    // }
}
