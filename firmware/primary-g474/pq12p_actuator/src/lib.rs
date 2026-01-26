//! High-level driver for an Actuonix PQ12-P linear actuator driven by a TI DRV8876 H-bridge,
//! built on top of `embassy-stm32` for `no_std` environments.
//!
//! # Overview
//!
//! This crate provides two actuator abstractions for the PQ12-P + DRV8876 combination:
//!
//! - [`Pq12P`] – uses *dual-PWM* drive (DRV8876 in 2×PWM mode, `IN1`/`IN2` both PWM-capable).
//!   Exposes explicit `coast`, `brake`, `move_out`, and `move_in` helpers, plus current sensing
//!   via the DRV8876 IPROPI pin.
//!
//! Both drivers are generic over `embassy-stm32` peripheral instances:
//!
//! - PWM: [`SimplePwmChannel`] on a [`GeneralInstance4Channel`] timer.
//! - ADC: [`Adc`] with [`AnyAdcChannel`] for position and/or current feedback.
//!
//! # Hardware assumptions
//!
//! - DRV8876 is wired in either:
//!   - 2×PWM mode (`IN1`/`IN2` both driven by timers), or
//!   - 1×PWM + direction GPIO mode.
//! - `nSLEEP` is held high during normal operation.
//! - ADC reference is 3.3 V and 12-bit (0–4095), matching `ADC_VREF` and `ADC_MAX_RAW`.
//! - The PQ12-P feedback pin is connected to an ADC channel, and (for [`Pq12P`]) the DRV8876
//! - The PQ12-P feedback reference voltages are 3V3 for P+ and GND for P-
//! - DRV8876 IPROPI pin is also connected to an ADC channel if current measurement is desired.
//!
//! TODO:
//! - [`SimplePq12P`] – uses a *single PWM* output plus a direction GPIO. Provides simple
//!   open-loop speed control and basic position feedback using the PQ12-P’s internal potentiometer,
//!   including a blocking [`SimplePq12P::move_to_pos`] primitive in millimeters.

#![no_std]

use embassy_stm32::{
    Peri,
    adc::{Adc, AnyAdcChannel, Instance, RxDma, SampleTime},
    timer::{GeneralInstance4Channel, simple_pwm::SimplePwmChannel},
};
use embassy_time::{Duration, Ticker};

/// Actuator Struct for the PQ12-P linear actuator paired with a DRV8876 for driving in 2 x PWM input mode.
///
/// DRV8876 H-bridge truth table (PWM mode, PMODE = 1).
///
/// In this mode `IN1` and `IN2` are independent logic/PWM inputs.
/// `nSLEEP = 1` is assumed during normal operation.
///
/// Motor / H-bridge behaviour:
///
/// | IN1 | IN2 | OUT1 | OUT2 | Description                          |
/// |-----|-----|------|------|--------------------------------------|
/// | 0   | 0   | Hi-Z | Hi-Z | Coast (both outputs high-impedance)  |
/// | 0   | 1   |  L   |  H   | Reverse drive (Plunger In)           |
/// | 1   | 0   |  H   |  L   | Forward drive (Plunger Out)          |
/// | 1   | 1   |  L   |  L   | Brake (slow-decay, both low-side on) |
pub struct Pq12P<'a, T: GeneralInstance4Channel, C: Instance> {
    pwm_1: SimplePwmChannel<'a, T>,
    pwm_2: SimplePwmChannel<'a, T>,
    pos_adc: AnyAdcChannel<C>,
    i_adc: AnyAdcChannel<C>,
}

impl<'a, T: GeneralInstance4Channel, C: Instance> Pq12P<'a, T, C> {
    pub const STROKE_LENGTH: f32 = 20.0;
    pub const PQ_POT_VREF: f32 = 3.3;
    pub const ADC_MAX_RAW: u16 = 4095;
    pub const ADC_VREF: f32 = 3.3;
    pub const R_IPROPI: f32 = 2490.0;

    /// Create a new instance of the PQ12-P & DRV8876 pair in 2 x PWM Input mode.
    pub fn new(
        pwm_1: SimplePwmChannel<'a, T>,
        pwm_2: SimplePwmChannel<'a, T>,
        pos_adc: AnyAdcChannel<C>,
        i_adc: AnyAdcChannel<C>,
    ) -> Self {
        Pq12P {
            pwm_1,
            pwm_2,
            pos_adc,
            i_adc,
        }
    }

    // === Movement Methods ===

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

    // === Async Sensor Reading Methods ===

    /// Read plunger position raw ADC value asynchronously.   
    pub async fn read_position_raw_async(
        &mut self,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) -> u16 {
        let mut reading = [0u16];
        adc.read(
            dma.reborrow(),
            [(&mut self.pos_adc, SampleTime::CYCLES12_5)].into_iter(),
            &mut reading,
        )
        .await;
        reading[0]
    }

    #[allow(non_snake_case)]
    /// Read plunger position as voltage asynchronously.
    pub async fn read_position_V_async(
        &mut self,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) -> f32 {
        let raw = self.read_position_raw_async(adc, dma).await;
        (raw as f32 / Self::ADC_MAX_RAW as f32) * Self::PQ_POT_VREF
    }

    /// Read plunger position in millimeters asynchronously.
    pub async fn read_position_mm_async(
        &mut self,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) -> f32 {
        let raw = self.read_position_raw_async(adc, dma).await;
        (raw as f32) / (Self::ADC_MAX_RAW as f32) * Self::STROKE_LENGTH
    }

    /// Read current sense pin of DRV8876 as a raw ADC value asynchronously.
    ///
    /// *Note: The DRV8876 is only able to read current draw when the PWM signal is high.
    /// As far as I am aware there is no way within embassy to synchronize ADC reads with a PWM timer.
    /// This leads to readings littered with 0s from when the PWM signal is low.
    /// No filtering is done by the function*    
    pub async fn read_current_raw_async(
        &mut self,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) -> u16 {
        let mut reading = [0u16];
        adc.read(
            dma.reborrow(),
            [(&mut self.i_adc, SampleTime::CYCLES12_5)].into_iter(),
            &mut reading,
        )
        .await;
        reading[0]
    }

    /// Read current sense pin of DRV8876 as a mA value asynchronously.
    ///
    /// *Note: The DRV8876 is only able to read current draw when the PWM signal is high.
    /// As far as I am aware there is no way within embassy to synchronize ADC reads with a PWM timer.
    /// This leads to readings littered with 0s from when the PWM signal is low.
    /// No filtering is done by the function*    
    #[allow(non_snake_case)]
    pub async fn read_current_mA_async(
        &mut self,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) -> f32 {
        let raw = self.read_current_raw_async(adc, dma).await;
        let volts = (raw as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF;
        (volts / Self::R_IPROPI) * 1e6
    }

    // === Async Motor Control Methods ===

    /// Command PQ12 to move to specified plunger location using a Bang-Bang Controller. Select end behaviour as coast or brake with `end_in_brake`.
    pub async fn move_to_position(
        &mut self,
        target_position_mm: f32,
        duty_cycle_percent: u8,
        end_in_brake: bool,
        adc: &mut Adc<'_, C>,
        dma: &mut Peri<'_, impl RxDma<C>>,
    ) {
        const TOLERANCE_MM: f32 = 0.1;
        const CONTROL_LOOP_HZ: u64 = 200;

        let target_pos = target_position_mm.clamp(1.0, Self::STROKE_LENGTH - 1.0);
        let mut current_pos = self.read_position_mm_async(adc, dma).await;

        if current_pos + TOLERANCE_MM < target_pos {
            self.move_out(duty_cycle_percent);
        } else if current_pos - TOLERANCE_MM > target_pos {
            self.move_in(duty_cycle_percent);
        } else {
            if end_in_brake == true {
                self.brake();
            } else {
                self.coast();
            }
            return;
        }

        let mut ticker: Ticker = Ticker::every(Duration::from_hz(CONTROL_LOOP_HZ));
        while (current_pos - target_pos).abs() > TOLERANCE_MM {
            current_pos = self.read_position_mm_async(adc, dma).await;
            ticker.next().await;
        }

        if end_in_brake == true {
            self.brake();
        } else {
            self.coast();
        }
    }

    // === Blocking Sensor Reading Methods ===

    /// Read plunger position raw ADC value blocking.     
    pub fn read_position_raw_blocking(&mut self, adc: &mut Adc<'_, C>) -> u16 {
        adc.blocking_read(&mut self.pos_adc)
    }

    #[allow(non_snake_case)]
    /// Read plunger position as voltage blocking.
    pub fn read_position_V_blocking(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw = self.read_position_raw_blocking(adc);
        (raw as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF
    }

    /// Read plunger position in millimeters blocking.
    pub fn read_position_mm_blocking(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw = self.read_position_raw_blocking(adc);
        (raw as f32) / (Self::ADC_MAX_RAW as f32) * Self::STROKE_LENGTH
    }

    /// Read current sense pin of DRV8876 as a raw ADC value blocking.
    ///
    /// *Note: The DRV8876 is only able to read current draw when the PWM signal is high.
    /// As far as I am aware there is no way within embassy to synchronize ADC reads with a PWM timer.
    /// This leads to readings littered with 0s from when the PWM signal is low.
    /// No filtering is done by the function*
    pub fn read_current_raw_blocking(&mut self, adc: &mut Adc<'_, C>) -> u16 {
        adc.blocking_read(&mut self.i_adc)
    }

    /// Read current sense pin of DRV8876 as a mA value blocking.
    ///
    /// *Note: The DRV8876 is only able to read current draw when the PWM signal is high.
    /// As far as I am aware there is no way within embassy to synchronize ADC reads with a PWM timer.
    /// This leads to readings littered with 0s from when the PWM signal is low.
    /// No filtering is done by the function*
    #[allow(non_snake_case)]
    pub fn read_current_mA_blocking(&mut self, adc: &mut Adc<'_, C>) -> f32 {
        let raw = self.read_current_raw_blocking(adc);
        let volts = (raw as f32 / Self::ADC_MAX_RAW as f32) * Self::ADC_VREF;
        (volts / Self::R_IPROPI) * 1e6
    }
}
