use crate::config::POSITION_COUNT;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PositionSensorId {
    Index1 = 0,
    Middle = 1,
    Ring = 2,
    Pinky = 3,
    ThumbFlex = 4,
    ThumbRevolve = 5,
    Index2 = 6,
    ThumbAux = 7,
}

// ADC sampling order in `main.rs` position reader channel list.
pub const POSITION_ADC_ORDER: [PositionSensorId; POSITION_COUNT] = [
    PositionSensorId::Index1,
    PositionSensorId::Middle,
    PositionSensorId::Ring,
    PositionSensorId::Pinky,
    PositionSensorId::ThumbFlex,
    PositionSensorId::ThumbRevolve,
    PositionSensorId::Index2,
    PositionSensorId::ThumbAux,
];

// Per-motor output polarity compensation.
// If true, ch1/ch2 duty values are swapped before writing to hardware.
#[derive(Clone, Copy)]
pub struct MotorPwmSwap {
    pub ring: bool,
    pub pinky: bool,
    pub thumb_flex: bool,
    pub index1: bool,
    pub middle: bool,
    pub thumb_revolve: bool,
}

pub const MOTOR_PWM_SWAP: MotorPwmSwap = MotorPwmSwap {
    ring: false,
    pinky: false,
    thumb_flex: false,
    index1: true,
    middle: false,
    thumb_revolve: false,
};
