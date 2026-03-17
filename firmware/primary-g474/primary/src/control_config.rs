use crate::config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};

pub const MAX_MOTORS: usize = 6;
pub const CONTROL_HZ: u64 = 200;
pub const CONTROL_DT_S: f32 = 1.0 / CONTROL_HZ as f32;
pub const OUTPUT_DEADBAND_PERCENT: u8 = 5;

// Logical motor indices (HRTIM A..F).
pub const MOTOR_INDEX_1: usize = 0; // HRTIM D
pub const MOTOR_MIDDLE: usize = 1; // HRTIM E
pub const MOTOR_RING: usize = 2; // HRTIM A
pub const MOTOR_PINKY: usize = 3; // HRTIM B
pub const MOTOR_THUMB_1: usize = 4; // HRTIM C
pub const MOTOR_THUMB_2: usize = 5; // TIM 1 CH 3/4 (revolve)

// Position/command index order:
// 0 index-1, 1 middle, 2 ring, 3 pinky, 4 thumb, 5 index-2, 6 thumb-2, 7 thumb-3.
pub const POS_INDEX_1: usize = 0; // PB11
pub const POS_MIDDLE: usize = 1; // PB1
pub const POS_RING: usize = 2; // PB0
pub const POS_PINKY: usize = 3; // PF0
pub const POS_THUMB_1: usize = 4; // PA2 (flex)
pub const POS_THUMB_2: usize = 5; // PA0 (revolve)
pub const POS_INDEX_2: usize = 6; // PA3 (USELESS)
pub const POS_THUMB_3: usize = 7; // PA1 (USELESS)

pub const MOTOR_NAMES: [&str; MAX_MOTORS] = [
    "ring",
    "pinky",
    "thumb-related",
    "index-1",
    "middle",
    "index-2",
];

pub const POSITION_NAMES: [&str; POSITION_COUNT] = [
    "index-1-pot",
    "middle-pot",
    "ring-pot",
    "pinky-pot",
    "thumb-pot-1",
    "index-2-pot",
    "thumb-pot-2",
    "thumb-pot-3",
];

pub const CMD_INDEX_1: usize = 0;
pub const CMD_MIDDLE: usize = 1;
pub const CMD_RING: usize = 2;
pub const CMD_PINKY: usize = 3;
pub const CMD_THUMB_1: usize = 4; // (flex)
pub const CMD_THUMB_2: usize = 5; // (revolve)

// Per-motor output polarity compensation.
// If true, ch1/ch2 duty values are swapped before writing to hardware.
pub const MOTOR_PWM_SWAP: [bool; MAX_MOTORS] = [
    true, // index-1
    false, // middle
    false, // ring
    false, // pinky
    false, // thumb-1
    false, // thumb-2
];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Position,
    Force,
}

// Flip this to switch the controller behavior.
pub const CONTROL_MODE: ControlMode = ControlMode::Position;

#[derive(Clone, Copy)]
pub struct PositionMap {
    pub motor_idx: usize,
    pub cmd_idx: usize,
    pub pos_idx: usize,
}

#[derive(Clone, Copy)]
pub struct ForceMap {
    pub motor_idx: usize,
    pub cmd_idx: usize,
    pub force_idx: usize,
}

pub const POSITION_MAPS: [PositionMap; MAX_MOTORS] = [
    PositionMap {
        motor_idx: MOTOR_RING,
        cmd_idx: CMD_RING,
        pos_idx: POS_RING,
    },
    PositionMap {
        motor_idx: MOTOR_PINKY,
        cmd_idx: CMD_PINKY,
        pos_idx: POS_PINKY,
    },
    PositionMap {
        motor_idx: MOTOR_THUMB_1,
        cmd_idx: CMD_THUMB_1,
        pos_idx: POS_THUMB_1,
    },
    PositionMap {
        motor_idx: MOTOR_INDEX_1,
        cmd_idx: CMD_INDEX_1,
        pos_idx: POS_INDEX_1,
    },
    PositionMap {
        motor_idx: MOTOR_MIDDLE,
        cmd_idx: CMD_MIDDLE,
        pos_idx: POS_MIDDLE,
    },
    PositionMap {
        motor_idx: MOTOR_THUMB_2,
        cmd_idx: CMD_THUMB_2,
        pos_idx: POS_THUMB_2,
    },
];

pub const FORCE_MAPS: [ForceMap; MAX_MOTORS-1] = [
    ForceMap {
        motor_idx: MOTOR_RING,
        cmd_idx: CMD_RING,
        force_idx: MOTOR_RING,
    },
    ForceMap {
        motor_idx: MOTOR_PINKY,
        cmd_idx: CMD_PINKY,
        force_idx: MOTOR_PINKY,
    },
    ForceMap {
        motor_idx: MOTOR_THUMB_1,
        cmd_idx: CMD_THUMB_1,
        force_idx: MOTOR_THUMB_1,
    },
    ForceMap {
        motor_idx: MOTOR_INDEX_1,
        cmd_idx: CMD_INDEX_1,
        force_idx: MOTOR_INDEX_1,
    },
    ForceMap {
        motor_idx: MOTOR_MIDDLE,
        cmd_idx: CMD_MIDDLE,
        force_idx: MOTOR_MIDDLE,
    },
    // no force control for thumb opp right now
    //
    // ForceMap {
    //     motor_idx: MOTOR_THUMB_2,
    //     cmd_idx: CMD_THUMB_2,
    //     force_idx: MOTOR_THUMB_2,
    // },
];

pub const POSITION_PID_GAINS: (f32, f32, f32) = (40.0, 0.0, 0.0);
pub const FORCE_PID_GAINS: (f32, f32, f32) = (100.0, 0.0, 0.0);

pub fn position_bits_to_mm(raw: u16) -> f32 {
    const STROKE_MM: f32 = 20.0;
    const ADC_MAX_RAW: f32 = 4095.0;
    (raw as f32) * (STROKE_MM / ADC_MAX_RAW)
}

pub fn force_bits_to_newtons(raw: u16) -> f32 {
    const MAX_FORCE: f32 = 4.9;
    const ADC_MAX_RAW: f32 = 4095.0;
    (raw as f32) * (MAX_FORCE / ADC_MAX_RAW)
}

pub const fn is_valid_config() -> bool {
    COMMAND_COUNT >= MAX_MOTORS && FORCE_COUNT >= MAX_MOTORS && POSITION_COUNT >= MAX_MOTORS
}
