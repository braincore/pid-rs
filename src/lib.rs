//! A proportional-integral-derivative (PID) controller.

#[derive(Debug)]
pub struct Pid {
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain.
    pub ki: f32,
    /// Derivative gain.
    pub kd: f32,
    /// Limit of contribution of P term: `(-p_limit <= P <= p_limit)`
    pub p_limit: f32,
    /// Limit of contribution of I term `(-i_limit <= I <= i_limit)`
    pub i_limit: f32,
    /// Limit of contribution of D term `(-d_limit <= D <= d_limit)`
    pub d_limit: f32,

    setpoint: Option<f32>,
    prev_measurement: Option<f32>,
    /// `integral_term = sum[error(t) * ki(t)] (for all t)`
    integral_term: f32,
}

#[derive(Debug)]
pub struct ControlOutput {
    /// Contribution of the P term to the output.
    pub p: f32,
    /// Contribution of the I term to the output.
    /// `i = sum[error(t) * ki(t)] (for all t)`
    pub i: f32,
    /// Contribution of the D term to the output.
    pub d: f32,
    /// Output of the PID controller.
    pub output: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32, p_limit: f32, i_limit: f32, d_limit: f32) -> Pid {
        Pid {
            kp,
            ki,
            kd,
            p_limit,
            i_limit,
            d_limit,
            setpoint: None,
            prev_measurement: None,
            integral_term: 0.0
        }
    }

    /// Sets the desired setpoint.
    ///
    /// Must be called before the first invocation to `next_control_output()`.
    pub fn update_setpoint(&mut self, setpoint: f32) {
        self.setpoint = Some(setpoint);
    }

    /// Resets the integral term back to zero. This may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) {
        self.integral_term = 0.0;
    }

    /// Given a new measurement, calculates the next control output.
    ///
    /// # Panics
    /// If a setpoint has not been set via `update_setpoint()`.
    pub fn next_control_output(&mut self, measurement: f32) -> ControlOutput {
        if self.setpoint.is_none() {
            panic!("No set point specified.");
        }
        let error = self.setpoint.unwrap() - measurement;

        let p_unbounded = error * self.kp;
        let p = self.p_limit.min(p_unbounded.abs()) * p_unbounded.signum();

        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        self.integral_term += error * self.ki;
        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.integral_term = self.i_limit.min(self.integral_term.abs()) * self.integral_term.signum();

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => {
                measurement - prev_measurement
            },
            None => {
                0.0
            }
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = self.d_limit.min(d_unbounded.abs()) * d_unbounded.signum();

        ControlOutput {
            p,
            i: self.integral_term,
            d,
            output: (p + self.integral_term + d)
        }
    }
}


#[cfg(test)]
mod tests {
    use super::Pid;

    #[test]
    fn proportional() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, 100.0, 100.0, 100.0);
        pid.update_setpoint(10.0);

        // Test simple proportional
        assert_eq!(pid.next_control_output(0.0).output, 20.0);

        // Test proportional limit
        pid.p_limit = 10.0;
        assert_eq!(pid.next_control_output(0.0).output, 10.0);
    }

    #[test]
    fn derivative() {
        let mut pid = Pid::new(0.0, 0.0, 2.0, 100.0, 100.0, 100.0);
        pid.update_setpoint(10.0);

        // Test that there's no derivative since it's the first measurement
        assert_eq!(pid.next_control_output(0.0).output, 0.0);

        // Test that there's now a derivative
        assert_eq!(pid.next_control_output(5.0).output, -10.0);

        // Test derivative limit
        pid.d_limit = 5.0;
        assert_eq!(pid.next_control_output(10.0).output, -5.0);
    }

    #[test]
    fn integral() {
        let mut pid = Pid::new(0.0, 2.0, 0.0, 100.0, 100.0, 100.0);
        pid.update_setpoint(10.0);

        // Test basic integration
        assert_eq!(pid.next_control_output(0.0).output, 20.0);
        assert_eq!(pid.next_control_output(0.0).output, 40.0);
        assert_eq!(pid.next_control_output(5.0).output, 50.0);

        // Test limit
        pid.i_limit = 50.0;
        assert_eq!(pid.next_control_output(5.0).output, 50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid.next_control_output(15.0).output, 40.0);

        // Test that error integral accumulates negative values
        let mut pid2 = Pid::new(0.0, 2.0, 0.0, 100.0, 100.0, 100.0);
        pid2.update_setpoint(-10.0);
        assert_eq!(pid2.next_control_output(0.0).output, -20.0);
        assert_eq!(pid2.next_control_output(0.0).output, -40.0);

        pid2.i_limit = 50.0;
        assert_eq!(pid2.next_control_output(-5.0).output, -50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid2.next_control_output(-15.0).output, -40.0);
    }

    #[test]
    fn pid() {
        let mut pid = Pid::new(1.0, 0.1, 1.0, 100.0, 100.0, 100.0);
        pid.update_setpoint(10.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0);  // 1.0 * 10.0
        assert_eq!(out.i, 1.0);  // 0.1 * 10.0
        assert_eq!(out.d, 0.0);  // -(1.0 * 0.0)
        assert_eq!(out.output, 11.0);

        let out = pid.next_control_output(5.0);
        assert_eq!(out.p, 5.0);  // 1.0 * 5.0
        assert_eq!(out.i, 1.5);  // 0.1 * (10.0 + 5.0)
        assert_eq!(out.d, -5.0);  // -(1.0 * 5.0)
        assert_eq!(out.output, 1.5);

        let out = pid.next_control_output(11.0);
        assert_eq!(out.p, -1.0);  // 1.0 * -1.0
        assert_eq!(out.i, 1.4);  // 0.1 * (10.0 + 5.0 - 1)
        assert_eq!(out.d, -6.0);  // -(1.0 * 6.0)
        assert_eq!(out.output, -5.6);

        let out = pid.next_control_output(10.0);
        assert_eq!(out.p, 0.0);  // 1.0 * 0.0
        assert_eq!(out.i, 1.4);  // 0.1 * (10.0 + 5.0 - 1.0 + 0.0)
        assert_eq!(out.d, 1.0);  // -(1.0 * -1.0)
        assert_eq!(out.output, 2.4);
    }
}
