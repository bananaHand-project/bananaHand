#[derive(Clone, Copy)]
pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integ: f32,
    prev_err: f32,
    integ_min: f32,
    integ_max: f32,
    out_min: f32,
    out_max: f32,
}

impl Pid {
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integ: 0.0,
            prev_err: 0.0,
            integ_min: -200.0,
            integ_max: 200.0,
            out_min: -100.0,
            out_max: 100.0,
        }
    }

    pub fn reset(&mut self) {
        self.integ = 0.0;
        self.prev_err = 0.0;
    }

    pub fn update(&mut self, setpoint: f32, measurement: f32, dt_s: f32) -> f32 {
        let err = setpoint - measurement;
        self.integ = (self.integ + err * dt_s).clamp(self.integ_min, self.integ_max);
        let deriv = (err - self.prev_err) / dt_s;
        self.prev_err = err;

        (self.kp * err + self.ki * self.integ + self.kd * deriv).clamp(self.out_min, self.out_max)
    }
}
