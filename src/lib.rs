//! A proportional-integral-derivative (PID) controller.
#![no_std]

use num_traits::float::FloatCore;
#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Pid<T: FloatCore> {
    /// Ideal setpoint to strive for
    pub setpoint: T,

    /// Proportional gain
    pub kp: T,

    /// Integral gain
    pub ki: T,

    /// Derivative gain
    pub kd: T,

    /// Limiter for the proportional term: `(-p_limit <= P <= p_limit)`
    pub p_limit: T,

    /// Limiter for the integral term: `(-i_limit <= I <= i_limit)`
    pub i_limit: T,

    /// Limiter for the derivative term: `(-d_limit <= D <= d_limit)`
    pub d_limit: T,

    /// Defines overall output filter limit
    pub output_limit: T,

    /// Last calculated integral value if [Pid::ki] is used
    integral_term: T,

    /// Previously found measurement whilst using the [Pid::next_control_output] method
    prev_measurement: Option<T>,
}

#[derive(Debug, PartialEq, Eq)]
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
    pub fn new(setpoint: impl Into<T>, limit: impl Into<T>) -> Self {
        Self {
            setpoint: setpoint.into(),
            kp: T::zero(),
            ki: T::zero(),
            kd: T::zero(),
            p_limit: T::zero(),
            i_limit: T::zero(),
            d_limit: T::zero(),
            output_limit: limit.into(),
            integral_term: T::zero(),
            prev_measurement: None,
        }
    }

    pub fn p(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kp = gain.into();
        self.p_limit = limit.into();
        self
    }

    pub fn i(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.ki = gain.into();
        self.i_limit = limit.into();
        self
    }

    pub fn d(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kd = gain.into();
        self.d_limit = limit.into();
        self
    }

    pub fn setpoint(&mut self, setpoint: impl Into<T>) -> &mut Self {
        self.setpoint = setpoint.into();
        self
    }

    /// Given a new measurement, calculates the next control output.
    ///
    /// # Panics
    /// If a setpoint has not been set via `update_setpoint()`.
    pub fn next_control_output(&mut self, measurement: T) -> ControlOutput<T> {
        let error = self.setpoint - measurement;

        let p_unbounded = error * self.kp;
        let p = apply_limit(self.p_limit, p_unbounded);

        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        self.integral_term = self.integral_term + error * self.ki;
        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.integral_term = apply_limit(self.i_limit, self.integral_term);

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => measurement - *prev_measurement,
            None => T::zero(),
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = apply_limit(self.d_limit, d_unbounded);

        let output = p + self.integral_term + d;
        let output = apply_limit(self.output_limit, output);

        ControlOutput {
            p,
            i: self.integral_term,
            d,
            output: output,
        }
    }

    /// Resets the integral term back to zero, this may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) {
        self.integral_term = T::zero();
    }
}

/// Saturating the input `value` according the absolute `limit` (`-limit <= output <= limit`).
fn apply_limit<T: FloatCore>(limit: T, value: T) -> T {
    limit.min(value.abs()) * value.signum()
}

#[cfg(test)]
mod tests {
    use crate::ControlOutput;

    use super::Pid;

    #[test]
    fn proportional() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(2.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);
        assert_eq!(pid.setpoint, 10.0);

        // Test simple proportional
        assert_eq!(pid.next_control_output(0.0).output, 20.0);

        // Test proportional limit
        pid.p_limit = 10.0;
        assert_eq!(pid.next_control_output(0.0).output, 10.0);
    }

    #[test]
    fn derivative() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i(0.0, 100.0).d(2.0, 100.0);

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
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(0.0, 100.0).i(2.0, 100.0).d(0.0, 100.0);

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
        let mut pid2 = Pid::new(-10.0, 100.0);
        pid2.p(0.0, 100.0).i(2.0, 100.0).d(0.0, 100.0);
        assert_eq!(pid2.next_control_output(0.0).output, -20.0);
        assert_eq!(pid2.next_control_output(0.0).output, -40.0);

        pid2.i_limit = 50.0;
        assert_eq!(pid2.next_control_output(-5.0).output, -50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid2.next_control_output(-15.0).output, -40.0);
    }

    #[test]
    fn output_limit() {
        let mut pid = Pid::new(10.0, 1.0);
        pid.p(1.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.output, 1.0);

        let out = pid.next_control_output(20.0);
        assert_eq!(out.p, -10.0); // 1.0 * (10.0 - 20.0)
        assert_eq!(out.output, -1.0);
    }

    #[test]
    fn pid() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

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
        let mut pid32 = Pid::new(10.0f32, 100.0);
        pid32.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        let mut pid64 = Pid::new(10.0, 100.0f64);
        pid64.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        assert_eq!(
            pid32.next_control_output(0.0).output,
            pid64.next_control_output(0.0).output as f32
        );
        assert_eq!(
            pid32.next_control_output(0.0).output as f64,
            pid64.next_control_output(0.0).output
        );
    }

    #[test]
    fn setpoint() {
        let mut pid = Pid::new(10.0, 100.0);
        pid.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0); // 1.0 * 10.0
        assert_eq!(out.i, 1.0); // 0.1 * 10.0
        assert_eq!(out.d, 0.0); // -(1.0 * 0.0)
        assert_eq!(out.output, 11.0);

        pid.setpoint(0.0);

        assert_eq!(
            pid.next_control_output(0.0),
            ControlOutput {
                p: 0.0,
                i: 1.0,
                d: -0.0,
                output: 1.0
            }
        );
    }
}
