use crate::control_config::{
    ACTUATOR_COUNT, ACTUATOR_LOOP_CONFIGS, CONTROL_DT_S, DEFAULT_CONTROL_MODE, CommandInputs,
    ControlMode, FORCE_PID_GAINS, ForceInputs, OUTPUT_DEADBAND_PERCENT, POSITION_PID_GAINS,
    PositionInputs, force_bits_to_newtons, position_bits_to_mm,
};
use crate::pid::Pid;

const RING_IDX: usize = 0;
const PINKY_IDX: usize = 1;
const THUMB_FLEX_IDX: usize = 2;
const INDEX1_IDX: usize = 3;
const MIDDLE_IDX: usize = 4;
const THUMB_REVOLVE_IDX: usize = 5;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum MotorCommand {
    Coast,
    Brake,
    MoveOut(u8),
    MoveIn(u8),
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MotorPwmCommand {
    pub ch1_percent: u8,
    pub ch2_percent: u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub struct MotorOutputs {
    pub ring: MotorCommand,
    pub pinky: MotorCommand,
    pub thumb_flex: MotorCommand,
    pub index1: MotorCommand,
    pub middle: MotorCommand,
    pub thumb_revolve: MotorCommand,
}

impl MotorOutputs {
    fn from_ordered(outputs: [MotorCommand; ACTUATOR_COUNT]) -> Self {
        Self {
            ring: outputs[RING_IDX],
            pinky: outputs[PINKY_IDX],
            thumb_flex: outputs[THUMB_FLEX_IDX],
            index1: outputs[INDEX1_IDX],
            middle: outputs[MIDDLE_IDX],
            thumb_revolve: outputs[THUMB_REVOLVE_IDX],
        }
    }
}

pub struct Controller {
    position_pids: [Pid; ACTUATOR_COUNT],
    force_pids: [Pid; ACTUATOR_COUNT],
    mode: ControlMode,
}

impl Controller {
    pub fn new() -> Self {
        // TODO: Fix this to be default instead and new should allow the dev to input their own gains
        let (pkp, pki, pkd) = POSITION_PID_GAINS;
        let (fkp, fki, fkd) = FORCE_PID_GAINS;
        Self {
            position_pids: core::array::from_fn(|_| Pid::new(pkp, pki, pkd)),
            force_pids: core::array::from_fn(|_| Pid::new(fkp, fki, fkd)),
            mode: DEFAULT_CONTROL_MODE,
        }
    }

    pub fn mode(&self) -> ControlMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: ControlMode) {
        if self.mode != mode {
            self.mode = mode;
            self.reset_all();
        }
    }

    fn reset_all(&mut self) {
        for pid in &mut self.position_pids {
            pid.reset();
        }
        for pid in &mut self.force_pids {
            pid.reset();
        }
    }

    pub fn step(
        &mut self,
        commands: &CommandInputs,
        positions: &PositionInputs,
        forces: &ForceInputs,
    ) -> MotorOutputs {
        match self.mode {
            ControlMode::Position => self.step_position(commands, positions),
            ControlMode::Force => self.step_force(commands, forces),
        }
    }

    fn step_position(
        &mut self,
        commands: &CommandInputs,
        positions: &PositionInputs,
    ) -> MotorOutputs {
        let mut outputs = [MotorCommand::Coast; ACTUATOR_COUNT];

        for (idx, actuator) in ACTUATOR_LOOP_CONFIGS.iter().copied().enumerate() {
            let setpoint_mm = position_bits_to_mm(commands.get(actuator.command));
            let feedback_mm = position_bits_to_mm(positions.get(actuator.position));
            let u = self.position_pids[idx].update(setpoint_mm, feedback_mm, CONTROL_DT_S);
            outputs[idx] = u.into();
        }

        MotorOutputs::from_ordered(outputs)
    }

    fn step_force(&mut self, commands: &CommandInputs, forces: &ForceInputs) -> MotorOutputs {
        let mut outputs = [MotorCommand::Brake; ACTUATOR_COUNT];

        for (idx, actuator) in ACTUATOR_LOOP_CONFIGS.iter().copied().enumerate() {
            let Some(force_channel) = actuator.force else {
                continue;
            };

            let setpoint = force_bits_to_newtons(commands.get(actuator.command));
            let feedback = force_bits_to_newtons(forces.get(force_channel));
            let u = self.force_pids[idx].update(setpoint, feedback, CONTROL_DT_S);
            outputs[idx] = u.into();
        }

        MotorOutputs::from_ordered(outputs)
    }
}

impl From<f32> for MotorCommand {
    fn from(u: f32) -> Self {
        let duty = u.abs().clamp(1.0, 100.0) as u8;
        if duty < OUTPUT_DEADBAND_PERCENT {
            return MotorCommand::Brake;
        }
        if u >= 0.0 {
            MotorCommand::MoveOut(duty)
        } else {
            MotorCommand::MoveIn(duty)
        }
    }
}

impl From<MotorCommand> for MotorPwmCommand {
    fn from(cmd: MotorCommand) -> Self {
        match cmd {
            MotorCommand::Coast => Self {
                ch1_percent: 0,
                ch2_percent: 0,
            },
            MotorCommand::Brake => Self {
                ch1_percent: 100,
                ch2_percent: 100,
            },
            MotorCommand::MoveOut(duty) => Self {
                ch1_percent: 100,
                ch2_percent: 101 - duty, // MUST BE 101 SO CH2 WILL NEVER BE 0, OR ELSE MOTOR JUST COASTS
            },
            MotorCommand::MoveIn(duty) => Self {
                ch1_percent: 101 - duty, // MUST BE 101 SO CH2 WILL NEVER BE 0, OR ELSE MOTOR JUST COASTS
                ch2_percent: 100,
            },
        }
    }
}
