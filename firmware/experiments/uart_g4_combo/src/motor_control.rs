use crate::config::{COMMAND_COUNT, FORCE_COUNT, POSITION_COUNT};
use crate::control_config::{
    CONTROL_DT_S, CONTROL_MODE, FORCE_MAPS, FORCE_PID_GAINS, MAX_MOTORS, OUTPUT_DEADBAND_PERCENT,
    POSITION_MAPS, POSITION_PID_GAINS, ControlMode, force_bits_to_newtons,
    HYBRID_DESIRED_FORCE_UNITS, HYBRID_FORCE_POSITION_GAINS, HYBRID_MAX_POSITION_OFFSET_MM,
    position_bits_to_mm,
};
use crate::pid::Pid;

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

pub struct Controller {
    position_pids: [Pid; MAX_MOTORS],
    force_pids: [Pid; MAX_MOTORS],
    mode: ControlMode,
}

impl Controller {
    pub fn new() -> Self {
        let (pkp, pki, pkd) = POSITION_PID_GAINS;
        let (fkp, fki, fkd) = FORCE_PID_GAINS;
        Self {
            position_pids: core::array::from_fn(|_| Pid::new(pkp, pki, pkd)),
            force_pids: core::array::from_fn(|_| Pid::new(fkp, fki, fkd)),
            mode: CONTROL_MODE,
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

    // MAIN PID LOGIC
    pub fn step(
        &mut self,
        commands: &[u16; COMMAND_COUNT],
        positions: &[u16; POSITION_COUNT],
        forces: &[u16; FORCE_COUNT],
    ) -> [MotorCommand; MAX_MOTORS] {
        let mut outputs = [MotorCommand::Coast; MAX_MOTORS];
        let mut active = [false; MAX_MOTORS];

        match self.mode {
            ControlMode::Position => {
                for map in POSITION_MAPS {
                    active[map.motor_idx] = true;
                    let setpoint_mm = position_bits_to_mm(commands[map.cmd_idx]);
                    let feedback_mm = position_bits_to_mm(positions[map.pos_idx]);
                    let u = self.position_pids[map.motor_idx].update(
                        setpoint_mm,
                        feedback_mm,
                        CONTROL_DT_S,
                    );
                    outputs[map.motor_idx] = motor_command_from_output(u);
                }
                apply_inactive_policy(&mut outputs, &active, ControlMode::Position);
            }
            ControlMode::Force => {
                for map in FORCE_MAPS {
                    active[map.motor_idx] = true;
                    let setpoint = force_bits_to_newtons(commands[map.cmd_idx]);
                    let feedback = force_bits_to_newtons(forces[map.force_idx]);
                    let u = self.force_pids[map.motor_idx].update(setpoint, feedback, CONTROL_DT_S);
                    // defmt::info!("setpoint: {}, force: {}, u: {}, idx: {}", setpoint, feedback, u, map.motor_idx);
                    outputs[map.motor_idx] = motor_command_from_output(u);
                }
                apply_inactive_policy(&mut outputs, &active, ControlMode::Force);
            }
            ControlMode::HybridForcePosition => {
                for map in POSITION_MAPS {
                    active[map.motor_idx] = true;

                    let base_setpoint_mm = position_bits_to_mm(commands[map.cmd_idx]);
                    let feedback_mm = position_bits_to_mm(positions[map.pos_idx]);

                    let force_feedback = FORCE_MAPS
                        .iter()
                        .find(|force_map| force_map.motor_idx == map.motor_idx)
                        .map(|force_map| force_bits_to_newtons(forces[force_map.force_idx]))
                        .unwrap_or(0.0);

                    let force_err = HYBRID_DESIRED_FORCE_UNITS[map.motor_idx] - force_feedback;
                    let position_offset_mm =
                        (HYBRID_FORCE_POSITION_GAINS[map.motor_idx] * force_err).clamp(
                            -HYBRID_MAX_POSITION_OFFSET_MM,
                            HYBRID_MAX_POSITION_OFFSET_MM,
                        );
                    let hybrid_setpoint_mm = base_setpoint_mm - position_offset_mm;

                    let u = self.position_pids[map.motor_idx].update(
                        hybrid_setpoint_mm,
                        feedback_mm,
                        CONTROL_DT_S,
                    );
                    outputs[map.motor_idx] = motor_command_from_output(u);
                }
                apply_inactive_policy(&mut outputs, &active, ControlMode::HybridForcePosition);
            }
        }

        outputs
    }
}

fn motor_command_from_output(u: f32) -> MotorCommand {
    let duty = u.abs().clamp(1.0, 100.0) as u8; // IF EITHER OF THE CHANNELS IS 0, MOTOR JUST COASTSE
    if duty < OUTPUT_DEADBAND_PERCENT {
        return MotorCommand::Brake;
    }
    if u >= 0.0 {
        MotorCommand::MoveOut(duty)
    } else {
        MotorCommand::MoveIn(duty)
    }
}

fn apply_inactive_policy(
    outputs: &mut [MotorCommand; MAX_MOTORS],
    active: &[bool; MAX_MOTORS],
    mode: ControlMode,
) {
    for i in 0..MAX_MOTORS {
        if active[i] {
            continue;
        }
        outputs[i] = match mode {
            ControlMode::Position => MotorCommand::Coast,
            ControlMode::Force => MotorCommand::Brake,
            ControlMode::HybridForcePosition => MotorCommand::Coast,
        };
    }
}

pub fn pwm_command_from_motor_command(cmd: MotorCommand) -> MotorPwmCommand {
    match cmd {
        MotorCommand::Coast => MotorPwmCommand {
            ch1_percent: 0,
            ch2_percent: 0,
        },
        MotorCommand::Brake => MotorPwmCommand {
            ch1_percent: 100,
            ch2_percent: 100,
        },
        // One side is held at 100%, the opposite side is 100 - duty.
        MotorCommand::MoveOut(duty) => MotorPwmCommand {
            ch1_percent: 100,
            ch2_percent: 101 - duty, // MUST BE 101 SO CH2 WILL NEVER BE 0, OR ELSE MOTOR JUST COASTS
        },
        MotorCommand::MoveIn(duty) => MotorPwmCommand {
            ch1_percent: 101 - duty, // MUST BE 101 SO CH1 WILL NEVER BE 0, OR ELSE MOTOR JUST COASTS (maybe)
            ch2_percent: 100,
        },
    }
}
