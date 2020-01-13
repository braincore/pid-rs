//! A proportional-integral-derivative (PID) controller.
#![no_std]

use num_traits::float::FloatCore;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub struct Pid<T: FloatCore> {
    /// Proportional gain.
    pub kp: T,
    /// Integral gain.
    pub ki: T,
    /// Derivative gain.
    pub kd: T,
    /// Limit of contribution of P term: `(-p_limit <= P <= p_limit)`
    pub p_limit: T,
    /// Limit of contribution of I term `(-i_limit <= I <= i_limit)`
    pub i_limit: T,
    /// Limit of contribution of D term `(-d_limit <= D <= d_limit)`
    pub d_limit: T,

    pub setpoint: T,
    prev_measurement: Option<T>,
    /// `integral_term = sum[error(t) * ki(t)] (for all t)`
    integral_term: T,
}

#[derive(Debug)]
pub struct ControlOutput<T: FloatCore> {
    /// Contribution of the P term to the output.
    pub p: T,
    /// Contribution of the I term to the output.
    /// `i = sum[error(t) * ki(t)] (for all t)`
    pub i: T,
    /// Contribution of the D term to the output.
    pub d: T,
    /// Output of the PID controller.
    pub output: T,
}

impl<T> Pid<T>
where
    T: FloatCore,
{
    pub fn new(kp: T, ki: T, kd: T, p_limit: T, i_limit: T, d_limit: T, setpoint: T) -> Self {
        Self {
            kp,
            ki,
            kd,
            p_limit,
            i_limit,
            d_limit,
            setpoint,
            prev_measurement: None,
            integral_term: T::zero(),
        }
    }

    /// Resets the integral term back to zero. This may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) {
        self.integral_term = T::zero();
    }

    /// Given a new measurement, calculates the next control output.
    ///
    /// # Panics
    /// If a setpoint has not been set via `update_setpoint()`.
    pub fn next_control_output(&mut self, measurement: T) -> ControlOutput<T> {
        let error = self.setpoint - measurement;

        let p_unbounded = error * self.kp;
        let p = self.p_limit.min(p_unbounded.abs()) * p_unbounded.signum();

        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        self.integral_term = self.integral_term + error * self.ki;
        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.integral_term =
            self.i_limit.min(self.integral_term.abs()) * self.integral_term.signum();

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => measurement - *prev_measurement,
            None => T::zero(),
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = self.d_limit.min(d_unbounded.abs()) * d_unbounded.signum();

        ControlOutput {
            p,
            i: self.integral_term,
            d,
            output: (p + self.integral_term + d),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Pid;

    #[test]
    fn proportional() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, 100.0, 100.0, 100.0, 10.0);
        assert_eq!(pid.setpoint, 10.0);

        // Test simple proportional
        assert_eq!(pid.next_control_output(0.0).output, 20.0);

        // Test proportional limit
        pid.p_limit = 10.0;
        assert_eq!(pid.next_control_output(0.0).output, 10.0);
    }

    #[test]
    fn derivative() {
        let mut pid = Pid::new(0.0, 0.0, 2.0, 100.0, 100.0, 100.0, 10.0);

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
        let mut pid = Pid::new(0.0, 2.0, 0.0, 100.0, 100.0, 100.0, 10.0);

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
        let mut pid2 = Pid::new(0.0, 2.0, 0.0, 100.0, 100.0, 100.0, -10.0);
        assert_eq!(pid2.next_control_output(0.0).output, -20.0);
        assert_eq!(pid2.next_control_output(0.0).output, -40.0);

        pid2.i_limit = 50.0;
        assert_eq!(pid2.next_control_output(-5.0).output, -50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid2.next_control_output(-15.0).output, -40.0);
    }

    #[test]
    fn pid() {
        let mut pid = Pid::new(1.0, 0.1, 1.0, 100.0, 100.0, 100.0, 10.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.i, 1.0); // 0.1 * 10.0
        assert_eq!(out.d, 0.0); // -(1.0 * 0.0)
        assert_eq!(out.output, 11.0);

        let out = pid.next_control_output(5.0);
        assert_eq!(out.p, 5.0); // 1.0 * 5.0
        assert_eq!(out.i, 1.5); // 0.1 * (10.0 + 5.0)
        assert_eq!(out.d, -5.0); // -(1.0 * 5.0)
        assert_eq!(out.output, 1.5);

        let out = pid.next_control_output(11.0);
        assert_eq!(out.p, -1.0); // 1.0 * -1.0
        assert_eq!(out.i, 1.4); // 0.1 * (10.0 + 5.0 - 1)
        assert_eq!(out.d, -6.0); // -(1.0 * 6.0)
        assert_eq!(out.output, -5.6);

        let out = pid.next_control_output(10.0);
        assert_eq!(out.p, 0.0); // 1.0 * 0.0
        assert_eq!(out.i, 1.4); // 0.1 * (10.0 + 5.0 - 1.0 + 0.0)
        assert_eq!(out.d, 1.0); // -(1.0 * -1.0)
        assert_eq!(out.output, 2.4);
    }

    #[test]
    fn f32_and_f64() {
        let mut pid32 = Pid::new(2.0f32, 0.0, 0.0, 100.0, 100.0, 100.0, 10.0);

        let mut pid64 = Pid::new(2.0f64, 0.0, 0.0, 100.0, 100.0, 100.0, 10.0);

        assert_eq!(
            pid32.next_control_output(0.0).output,
            pid64.next_control_output(0.0).output as f32
        );
        assert_eq!(
            pid32.next_control_output(0.0).output as f64,
            pid64.next_control_output(0.0).output
        );
    }

}
