use crate::config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};

pub const MAX_MOTORS: usize = 6;
pub const CONTROL_HZ: u64 = 200;
pub const CONTROL_DT_S: f32 = 1.0 / CONTROL_HZ as f32;
pub const OUTPUT_DEADBAND_PERCENT: u8 = 3;

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

// TODO: Factor this into more readable format with joint names

pub const POSITION_MAPS: [PositionMap; MAX_MOTORS] = [
    PositionMap { motor_idx: 0, cmd_idx: 0, pos_idx: 0 },
    PositionMap { motor_idx: 1, cmd_idx: 1, pos_idx: 1 },
    PositionMap { motor_idx: 2, cmd_idx: 2, pos_idx: 2 },
    PositionMap { motor_idx: 3, cmd_idx: 3, pos_idx: 3 },
    PositionMap { motor_idx: 4, cmd_idx: 4, pos_idx: 4 },
    PositionMap { motor_idx: 5, cmd_idx: 5, pos_idx: 5 },
];

pub const FORCE_MAPS: [ForceMap; MAX_MOTORS] = [
    ForceMap { motor_idx: 0, cmd_idx: 0, force_idx: 0 },
    ForceMap { motor_idx: 1, cmd_idx: 1, force_idx: 1 },
    ForceMap { motor_idx: 2, cmd_idx: 2, force_idx: 2 },
    ForceMap { motor_idx: 3, cmd_idx: 3, force_idx: 3 },
    ForceMap { motor_idx: 4, cmd_idx: 4, force_idx: 4 },
    ForceMap { motor_idx: 5, cmd_idx: 5, force_idx: 5 },
];

pub const POSITION_PID_GAINS: (f32, f32, f32) = (40.0, 0.0, 0.0);
pub const FORCE_PID_GAINS: (f32, f32, f32) = (0.08, 0.0, 0.0);

pub fn command_raw_to_position_mm(raw: u16) -> f32 {
    // Map full 16-bit command space to 0..20 mm stroke.
    20.0 * (raw as f32) / (u16::MAX as f32)
}

pub fn command_raw_to_force_units(raw: u16) -> f32 {
    // Placeholder until force command units are finalized.
    raw as f32
}

pub fn force_raw_to_units(raw: u16) -> f32 {
    // Placeholder calibration for force mode; replace with sensor calibration.
    raw as f32
}

pub const fn is_valid_config() -> bool {
    COMMAND_COUNT >= MAX_MOTORS && FORCE_COUNT >= MAX_MOTORS && POSITION_COUNT >= MAX_MOTORS
}
