use embassy_stm32::Peri;
use embassy_stm32::adc::{AdcChannel, AnyAdcChannel};
use embassy_stm32::peripherals::{
    ADC1, ADC2, PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PB0, PB1, PB2, PB11,
    PB12, PB13, PB14, PB15, PC4, PC5, PC6, PC7, PC8, PC9, PF0, PF1, TIM1,
};
use embassy_stm32::timer::simple_pwm::SimplePwmChannel;
use hrtim_pwm_hal::{
    SubtimerAPwm, SubtimerBPwm, SubtimerCPwm, SubtimerDPwm, SubtimerEPwm, SubtimerFPwm,
};

use crate::config::{CURRENT_COUNT, POSITION_COUNT};
use crate::motor_control::MotorPwmOutputs;

pub type RingPwm = SubtimerAPwm<PA8, PA9>;
pub type PinkyPwm = SubtimerBPwm<PA10, PA11>;
pub type ThumbFlexPwm = SubtimerCPwm<PB12, PB13>;
pub type Index1Pwm = SubtimerDPwm<PB14, PB15>;
pub type MiddlePwm = SubtimerEPwm<PC8, PC9>;
pub type Index2UnusedPwm = SubtimerFPwm<PC6, PC7>;
pub type Tim1PwmChannel = SimplePwmChannel<'static, TIM1>;

pub struct MotorPwm {
    ring: RingPwm,
    pinky: PinkyPwm,
    thumb_flex: ThumbFlexPwm,
    index1: Index1Pwm,
    middle: MiddlePwm,
    _index2_unused: Index2UnusedPwm,
    thumb_revolve_ch1: Tim1PwmChannel,
    thumb_revolve_ch2: Tim1PwmChannel,
    _thumb_flex_unused_ch1: Tim1PwmChannel,
    _thumb_flex_unused_ch2: Tim1PwmChannel,
}

impl MotorPwm {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        ring: RingPwm,
        pinky: PinkyPwm,
        thumb_flex: ThumbFlexPwm,
        index1: Index1Pwm,
        middle: MiddlePwm,
        index2_unused: Index2UnusedPwm,
        // TIM1 channel contract expected from caller:
        // CH1=thumb_revolve_ch1, CH2=thumb_revolve_ch2, CH3/CH4 currently unused.
        mut thumb_revolve_ch1: Tim1PwmChannel,
        mut thumb_revolve_ch2: Tim1PwmChannel,
        mut thumb_flex_unused_ch1: Tim1PwmChannel,
        mut thumb_flex_unused_ch2: Tim1PwmChannel,
    ) -> Self {
        ring.ch1_en();
        ring.ch2_en();
        pinky.ch1_en();
        pinky.ch2_en();
        thumb_flex.ch1_en();
        thumb_flex.ch2_en();
        index1.ch1_en();
        index1.ch2_en();
        middle.ch1_en();
        middle.ch2_en();
        index2_unused.ch1_en();
        index2_unused.ch2_en();
        thumb_revolve_ch1.enable();
        thumb_revolve_ch2.enable();
        thumb_flex_unused_ch1.enable();
        thumb_flex_unused_ch2.enable();

        Self {
            ring,
            pinky,
            thumb_flex,
            index1,
            middle,
            _index2_unused: index2_unused,
            thumb_revolve_ch1,
            thumb_revolve_ch2,
            _thumb_flex_unused_ch1: thumb_flex_unused_ch1,
            _thumb_flex_unused_ch2: thumb_flex_unused_ch2,
        }
    }

    pub fn apply_outputs(&mut self, outputs: MotorPwmOutputs) {
        self.ring.ch1_set_dc_percent(outputs.ring.ch1_percent);
        self.ring.ch2_set_dc_percent(outputs.ring.ch2_percent);

        self.pinky.ch1_set_dc_percent(outputs.pinky.ch1_percent);
        self.pinky.ch2_set_dc_percent(outputs.pinky.ch2_percent);

        self.thumb_flex
            .ch1_set_dc_percent(outputs.thumb_flex.ch1_percent);
        self.thumb_flex
            .ch2_set_dc_percent(outputs.thumb_flex.ch2_percent);

        // INDEX1 has ch1/ch2 swapped in this board revision's schematic.
        self.index1.ch1_set_dc_percent(outputs.index1.ch2_percent);
        self.index1.ch2_set_dc_percent(outputs.index1.ch1_percent);

        self.middle.ch1_set_dc_percent(outputs.middle.ch1_percent);
        self.middle.ch2_set_dc_percent(outputs.middle.ch2_percent);

        self.thumb_revolve_ch1
            .set_duty_cycle_percent(outputs.thumb_revolve.ch1_percent);
        self.thumb_revolve_ch2
            .set_duty_cycle_percent(outputs.thumb_revolve.ch2_percent);
    }
}

pub struct PositionAdcPins {
    pinky: Peri<'static, PF0>,
    ring: Peri<'static, PB0>,
    middle: Peri<'static, PB1>,
    index1: Peri<'static, PA3>,
    index2: Peri<'static, PB11>,
    thumb_flex: Peri<'static, PA2>,
    thumb_opp: Peri<'static, PA0>,
    thumb_aux: Peri<'static, PA1>,
}

pub fn position_adc_pins(
    pa0: Peri<'static, PA0>,
    pa1: Peri<'static, PA1>,
    pa2: Peri<'static, PA2>,
    pa3: Peri<'static, PA3>,
    pb0: Peri<'static, PB0>,
    pb1: Peri<'static, PB1>,
    pb11: Peri<'static, PB11>,
    pf0: Peri<'static, PF0>,
) -> PositionAdcPins {
    PositionAdcPins {
        pinky: pf0,
        ring: pb0,
        middle: pb1,
        index1: pa3,
        index2: pb11,
        thumb_flex: pa2,
        thumb_opp: pa0,
        thumb_aux: pa1,
    }
}

impl PositionAdcPins {
    pub fn into_ordered_channels(self) -> [AnyAdcChannel<'static, ADC1>; POSITION_COUNT] {
        [
            self.index1.degrade_adc(),     // [0]
            self.middle.degrade_adc(),     // [1]
            self.ring.degrade_adc(),       // [2]
            self.pinky.degrade_adc(),      // [3]
            self.thumb_flex.degrade_adc(), // [4]
            self.thumb_opp.degrade_adc(),  // [5]
            self.index2.degrade_adc(),     // [6]
            self.thumb_aux.degrade_adc(),  // [7]
        ]
    }
}

pub struct CurrentAdcPins {
    index1: Peri<'static, PB2>,
    middle: Peri<'static, PC5>,
    ring: Peri<'static, PA7>,
    pinky: Peri<'static, PA6>,
    thumb_flex: Peri<'static, PA5>,
    thumb_revolve: Peri<'static, PF1>,
    index2: Peri<'static, PC4>,
    thumb_aux: Peri<'static, PA4>,
}

pub fn current_adc_pins(
    pa4: Peri<'static, PA4>,
    pa5: Peri<'static, PA5>,
    pa6: Peri<'static, PA6>,
    pa7: Peri<'static, PA7>,
    pb2: Peri<'static, PB2>,
    pc4: Peri<'static, PC4>,
    pc5: Peri<'static, PC5>,
    pf1: Peri<'static, PF1>,
) -> CurrentAdcPins {
    CurrentAdcPins {
        index1: pb2,
        middle: pc5,
        ring: pa7,
        pinky: pa6,
        thumb_flex: pa5,
        thumb_revolve: pf1,
        index2: pc4,
        thumb_aux: pa4,
    }
}

impl CurrentAdcPins {
    pub fn into_ordered_channels(self) -> [AnyAdcChannel<'static, ADC2>; CURRENT_COUNT] {
        [
            self.index1.degrade_adc(),        // [0]
            self.middle.degrade_adc(),        // [1]
            self.ring.degrade_adc(),          // [2]
            self.pinky.degrade_adc(),         // [3]
            self.thumb_flex.degrade_adc(),    // [4]
            self.thumb_revolve.degrade_adc(), // [5]
            self.index2.degrade_adc(),        // [6]
            self.thumb_aux.degrade_adc(),     // [7]
        ]
    }
}
