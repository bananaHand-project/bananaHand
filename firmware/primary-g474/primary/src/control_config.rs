use crate::config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};
use banana_hand_common::{
    FORCE_SENSOR_INDEX_IDX, FORCE_SENSOR_MIDDLE_IDX, FORCE_SENSOR_PALM_1_IDX, FORCE_SENSOR_PALM_2_IDX,
    FORCE_SENSOR_PALM_3_IDX, FORCE_SENSOR_PALM_4_IDX, FORCE_SENSOR_PALM_5_IDX, FORCE_SENSOR_PINKY_IDX,
    FORCE_SENSOR_RING_IDX, FORCE_SENSOR_THUMB_IDX,
};

pub const CONTROL_HZ: u64 = 200;
pub const CONTROL_DT_S: f32 = 1.0 / CONTROL_HZ as f32;
pub const OUTPUT_DEADBAND_PERCENT: u8 = 5;

pub const ACTUATOR_COUNT: usize = 6;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Position,
    Force,
}

pub const DEFAULT_CONTROL_MODE: ControlMode = ControlMode::Position;

impl ControlMode {
    pub const fn from_wire(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Position),
            1 => Some(Self::Force),
            _ => None,
        }
    }

    pub const fn to_wire(self) -> u8 {
        match self {
            Self::Position => 0,
            Self::Force => 1,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CommandChannel {
    Ring,
    Pinky,
    ThumbFlex,
    Index1,
    Middle,
    ThumbRevolve,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum PositionChannel {
    Ring,
    Pinky,
    ThumbFlex,
    Index1,
    Middle,
    ThumbRevolve,
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum ForceChannel {
    Ring,
    Pinky,
    Thumb,
    Index,
    Middle,
    Palm1,
    Palm2,
    Palm3,
    Palm4,
    Palm5,
}

#[derive(Clone, Copy)]
pub struct CommandInputs {
    pub index1: u16,
    pub middle: u16,
    pub ring: u16,
    pub pinky: u16,
    pub thumb_flex: u16,
    pub thumb_revolve: u16,
}

impl CommandInputs {
    pub fn from_raw(raw: &[u16; COMMAND_COUNT]) -> Self {
        Self {
            index1: raw[0],
            middle: raw[1],
            ring: raw[2],
            pinky: raw[3],
            thumb_flex: raw[4],
            thumb_revolve: raw[5],
        }
    }

    pub fn get(&self, channel: CommandChannel) -> u16 {
        match channel {
            CommandChannel::Ring => self.ring,
            CommandChannel::Pinky => self.pinky,
            CommandChannel::ThumbFlex => self.thumb_flex,
            CommandChannel::Index1 => self.index1,
            CommandChannel::Middle => self.middle,
            CommandChannel::ThumbRevolve => self.thumb_revolve,
        }
    }
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
pub struct PositionInputs {
    pub index1: u16,
    pub middle: u16,
    pub ring: u16,
    pub pinky: u16,
    pub thumb_flex: u16,
    pub thumb_revolve: u16,
    pub index2: u16,
    pub thumb_aux: u16,
}

impl PositionInputs {
    pub fn from_raw(raw: &[u16; POSITION_COUNT]) -> Self {
        Self {
            index1: raw[0],
            middle: raw[1],
            ring: raw[2],
            pinky: raw[3],
            thumb_flex: raw[4],
            thumb_revolve: raw[5],
            index2: raw[6],
            thumb_aux: raw[7],
        }
    }

    pub fn get(&self, channel: PositionChannel) -> u16 {
        match channel {
            PositionChannel::Ring => self.ring,
            PositionChannel::Pinky => self.pinky,
            PositionChannel::ThumbFlex => self.thumb_flex,
            PositionChannel::Index1 => self.index1,
            PositionChannel::Middle => self.middle,
            PositionChannel::ThumbRevolve => self.thumb_revolve,
        }
    }
}

#[derive(Clone, Copy)]
#[allow(dead_code)]
pub struct ForceInputs {
    pub index: u16,
    pub middle: u16,
    pub ring: u16,
    pub pinky: u16,
    pub thumb: u16,
    pub palm_1: u16,
    pub palm_2: u16,
    pub palm_3: u16,
    pub palm_4: u16,
    pub palm_5: u16,
}

impl ForceInputs {
    pub fn from_raw(raw: &[u16; FORCE_COUNT]) -> Self {
        Self {
            index: raw[FORCE_SENSOR_INDEX_IDX],
            middle: raw[FORCE_SENSOR_MIDDLE_IDX],
            ring: raw[FORCE_SENSOR_RING_IDX],
            pinky: raw[FORCE_SENSOR_PINKY_IDX],
            thumb: raw[FORCE_SENSOR_THUMB_IDX],
            palm_1: raw[FORCE_SENSOR_PALM_1_IDX],
            palm_2: raw[FORCE_SENSOR_PALM_2_IDX],
            palm_3: raw[FORCE_SENSOR_PALM_3_IDX],
            palm_4: raw[FORCE_SENSOR_PALM_4_IDX],
            palm_5: raw[FORCE_SENSOR_PALM_5_IDX],
        }
    }

    pub fn get(&self, channel: ForceChannel) -> u16 {
        match channel {
            ForceChannel::Ring => self.ring,
            ForceChannel::Pinky => self.pinky,
            ForceChannel::Thumb => self.thumb,
            ForceChannel::Index => self.index,
            ForceChannel::Middle => self.middle,
            ForceChannel::Palm1 => self.palm_1,
            ForceChannel::Palm2 => self.palm_2,
            ForceChannel::Palm3 => self.palm_3,
            ForceChannel::Palm4 => self.palm_4,
            ForceChannel::Palm5 => self.palm_5,
        }
    }
}

#[derive(Clone, Copy)]
pub struct ActuatorLoopConfig {
    pub command: CommandChannel,
    pub position: PositionChannel,
    pub force: Option<ForceChannel>,
}

// Actuator order used by the control core and MotorOutputs:
// [ring, pinky, thumb_flex, index1, middle, thumb_revolve]
pub const ACTUATOR_LOOP_CONFIGS: [ActuatorLoopConfig; ACTUATOR_COUNT] = [
    ActuatorLoopConfig {
        command: CommandChannel::Ring,
        position: PositionChannel::Ring,
        force: Some(ForceChannel::Ring),
    },
    ActuatorLoopConfig {
        command: CommandChannel::Pinky,
        position: PositionChannel::Pinky,
        force: Some(ForceChannel::Pinky),
    },
    ActuatorLoopConfig {
        command: CommandChannel::ThumbFlex,
        position: PositionChannel::ThumbFlex,
        force: Some(ForceChannel::Thumb),
    },
    ActuatorLoopConfig {
        command: CommandChannel::Index1,
        position: PositionChannel::Index1,
        force: Some(ForceChannel::Index),
    },
    ActuatorLoopConfig {
        command: CommandChannel::Middle,
        position: PositionChannel::Middle,
        force: Some(ForceChannel::Middle),
    },
    ActuatorLoopConfig {
        command: CommandChannel::ThumbRevolve,
        position: PositionChannel::ThumbRevolve,
        force: None, // no force control for thumb-revolve right now
    },
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
    COMMAND_COUNT >= 6 && FORCE_COUNT >= 10 && POSITION_COUNT >= 8
}
