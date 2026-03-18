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
