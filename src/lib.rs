//! A proportional-integral-derivative (PID) controller library.
//!
//! See [Pid] for the adjustable controller itself, as well as [ControlOutput] for the outputs and weights which you can use after setting up your controller. Follow the complete example below to setup your first controller!
//!
//! # Example
//!
//! ```rust
//! use pid::Pid;
//!
//! // Create a new proportional-only PID controller with a setpoint of 15
//! let mut pid = Pid::new(15.0, 100.0);
//! pid.p(10.0, 100.0);
//!
//! // Input a measurement with an error of 5.0 from our setpoint
//! let output = pid.next_control_output(10.0);
//!
//! // Show that the error is correct by multiplying by our kp
//! assert_eq!(output.output, 50.0); // <--
//! assert_eq!(output.p, 50.0);
//!
//! // It won't change on repeat; the controller is proportional-only
//! let output = pid.next_control_output(10.0);
//! assert_eq!(output.output, 50.0); // <--
//! assert_eq!(output.p, 50.0);
//!
//! // Add a new integral term to the controller and input again
//! pid.i(1.0, 100.0);
//! let output = pid.next_control_output(10.0);
//!
//! // Now that the integral makes the controller stateful, it will change
//! assert_eq!(output.output, 55.0); // <--
//! assert_eq!(output.p, 50.0);
//! assert_eq!(output.i, 5.0);
//!
//! // Add our final derivative term and match our setpoint target
//! pid.d(2.0, 100.0);
//! let output = pid.next_control_output(15.0);
//!
//! // The output will now say to go down due to the derivative
//! assert_eq!(output.output, -5.0); // <--
//! assert_eq!(output.p, 0.0);
//! assert_eq!(output.i, 5.0);
//! assert_eq!(output.d, -10.0);
//! ```
#![no_std]

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A trait for any numeric type usable in the PID controller
///
/// This trait is automatically implemented for all types that satisfy `PartialOrd + num_traits::Signed + Copy`. This includes all of the signed float types and builtin integer except for [isize]:
/// - [i8]
/// - [i16]
/// - [i32]
/// - [i64]
/// - [i128]
/// - [f32]
/// - [f64]
///
/// As well as any user type that matches the requirements
pub trait Number: PartialOrd + num_traits::Signed + Copy {}

// Implement `Number` for all types that
// satisfy `PartialOrd + num_traits::Signed + Copy`.
impl<T: PartialOrd + num_traits::Signed + Copy> Number for T {}

/// Adjustable proportional-integral-derivative (PID) controller.
///
/// # Examples
///
/// This controller provides a builder pattern interface which allows you to pick-and-choose which PID inputs you'd like to use during operation. Here's what a basic proportional-only controller could look like:
///
/// ```rust
/// use pid::Pid;
///
/// // Create limited controller
/// let mut p_controller = Pid::new(15.0, 100.0);
/// p_controller.p(10.0, 100.0);
///
/// // Get first output
/// let p_output = p_controller.next_control_output(400.0);
/// ```
///
/// This controller would give you set a proportional controller to `10.0` with a target of `15.0` and an output limit of `100.0` per [output](Self::next_control_output) iteration. The same controller with a full PID system built in looks like:
///
/// ```rust
/// use pid::Pid;
///
/// // Create full PID controller
/// let mut full_controller = Pid::new(15.0, 100.0);
/// full_controller.p(10.0, 100.0).i(4.5, 100.0).d(0.25, 100.0);
///
/// // Get first output
/// let full_output = full_controller.next_control_output(400.0);
/// ```
///
/// This [`next_control_output`](Self::next_control_output) method is what's used to input new values into the controller to tell it what the current state of the system is. In the examples above it's only being used once, but realistically this will be a hot method. Please see [ControlOutput] for examples of how to handle these outputs; it's quite straight forward and mirrors the values of this structure in some ways.
///
/// The last item of note is that these [`p`](Self::p()), [`i`](Self::i()), and [`d`](Self::d()) methods can be used *during* operation which lets you add and/or modify these controller values if need be.
///
/// # Type Warning
///
/// [Number] is abstract and can be used with anything from a [i32] to an [i128] (as well as user-defined types). Because of this, very small types might overflow during calculation in [`next_control_output`](Self::next_control_output). You probably don't want to use [i8] or user-defined types around that size so keep that in mind when designing your controller.
#[derive(Clone, Copy, Eq, PartialEq, Ord, PartialOrd)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
pub struct Pid<T: Number> {
    /// Ideal setpoint to strive for.
    pub setpoint: T,
    /// Defines the overall output filter limit.
    pub output_limit: T,
    /// Proportional gain.
    pub kp: T,
    /// Integral gain.
    pub ki: T,
    /// Derivative gain.
    pub kd: T,
    /// Limiter for the proportional term: `-p_limit <= P <= p_limit`.
    pub p_limit: T,
    /// Limiter for the integral term: `-i_limit <= I <= i_limit`.
    pub i_limit: T,
    /// Limiter for the derivative term: `-d_limit <= D <= d_limit`.
    pub d_limit: T,
    /// Last calculated integral value if [Pid::ki] is used.
    integral_term: T,
    /// Previously found measurement whilst using the [Pid::next_control_output] method.
    prev_measurement: Option<T>,
}

/// Output of [controller iterations](Pid::next_control_output) with weights
///
/// # Example
///
/// This structure is simple to use and features three weights: [p](Self::p), [i](Self::i), and [d](Self::d). These can be used to figure out how much each term from [Pid] contributed to the final [output](Self::output) value which should be taken as the final controller output for this iteration:
///
/// ```rust
/// use pid::{Pid, ControlOutput};
///
/// // Setup controller
/// let mut pid = Pid::new(15.0, 100.0);
/// pid.p(10.0, 100.0).i(1.0, 100.0).d(2.0, 100.0);
///
/// // Input an example value and get a report for an output iteration
/// let output = pid.next_control_output(26.2456);
/// println!("P: {}\nI: {}\nD: {}\nFinal Output: {}", output.p, output.i, output.d, output.output);
/// ```
#[derive(Debug, PartialEq, Eq)]
pub struct ControlOutput<T: Number> {
    /// Contribution of the P term to the output.
    pub p: T,
    /// Contribution of the I term to the output.
    ///
    /// This integral term is equal to `sum[error(t) * ki(t)] (for all t)`
    pub i: T,
    /// Contribution of the D term to the output.
    pub d: T,
    /// Output of the PID controller.
    pub output: T,
}

impl<T> Pid<T>
where
    T: Number,
{
    /// Creates a new controller with the target setpoint and the output limit
    ///
    /// To set your P, I, and D terms into this controller, please use the following builder methods:
    /// - [Self::p()]: Proportional term setting
    /// - [Self::i()]: Integral term setting
    /// - [Self::d()]: Derivative term setting
    pub fn new(setpoint: impl Into<T>, output_limit: impl Into<T>) -> Self {
        Self {
            setpoint: setpoint.into(),
            output_limit: output_limit.into(),
            kp: T::zero(),
            ki: T::zero(),
            kd: T::zero(),
            p_limit: T::zero(),
            i_limit: T::zero(),
            d_limit: T::zero(),
            integral_term: T::zero(),
            prev_measurement: None,
        }
    }

    /// Sets the [Self::p] term for this controller.
    pub fn p(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kp = gain.into();
        self.p_limit = limit.into();
        self
    }

    /// Sets the [Self::i] term for this controller.
    pub fn i(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.ki = gain.into();
        self.i_limit = limit.into();
        self
    }

    /// Sets the [Self::d] term for this controller.
    pub fn d(&mut self, gain: impl Into<T>, limit: impl Into<T>) -> &mut Self {
        self.kd = gain.into();
        self.d_limit = limit.into();
        self
    }

    /// Sets the [Pid::setpoint] to target for this controller.
    pub fn setpoint(&mut self, setpoint: impl Into<T>) -> &mut Self {
        self.setpoint = setpoint.into();
        self
    }

    /// Given a new measurement, calculates the next [control output](ControlOutput).
    ///
    /// # Panics
    ///
    /// - If a setpoint has not been set via `update_setpoint()`.
    /// Modify existing methods into thin wrappers
    pub fn next_control_output(&mut self, measurement: T) -> ControlOutput<T> {
        // Wrapper at dt = 1.0 (maintains backward compatibility)
        self.next_control_output_with_dt(measurement, T::one())
    }
    // pub fn next_control_output(&mut self, measurement: T) -> ControlOutput<T> {
    //     // Calculate the error between the ideal setpoint and the current
    //     // measurement to compare against
    //     let error = self.setpoint - measurement;

    //     // Calculate the proportional term and limit to it's individual limit
    //     let p_unbounded = error * self.kp;
    //     let p = apply_limit(self.p_limit, p_unbounded);

    //     // Mitigate output jumps when ki(t) != ki(t-1).
    //     // While it's standard to use an error_integral that's a running sum of
    //     // just the error (no ki), because we support ki changing dynamically,
    //     // we store the entire term so that we don't need to remember previous
    //     // ki values.
    //     self.integral_term = self.integral_term + error * self.ki;

    //     // Mitigate integral windup: Don't want to keep building up error
    //     // beyond what i_limit will allow.
    //     self.integral_term = apply_limit(self.i_limit, self.integral_term);

    //     // Mitigate derivative kick: Use the derivative of the measurement
    //     // rather than the derivative of the error.
    //     let d_unbounded = -match self.prev_measurement.as_ref() {
    //         Some(prev_measurement) => measurement - *prev_measurement,
    //         None => T::zero(),
    //     } * self.kd;
    //     self.prev_measurement = Some(measurement);
    //     let d = apply_limit(self.d_limit, d_unbounded);

    //     // Calculate the final output by adding together the PID terms, then
    //     // apply the final defined output limit
    //     let output = p + self.integral_term + d;
    //     let output = apply_limit(self.output_limit, output);

    //     // Return the individual term's contributions and the final output
    //     ControlOutput {
    //         p,
    //         i: self.integral_term,
    //         d,
    //         output,
    //     }
    // }

    /// Resets the integral term back to zero, this may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) {
        self.integral_term = T::zero();
    }

    /// Set integral term to custom value.
    /// This may drastically change the control output.
    /// Use this to return the PID controller to a previous state after an interruption or crash.
    pub fn set_integral_term(&mut self, integral_term: impl Into<T>) -> &mut Self {
        self.integral_term = integral_term.into();
        self
    }

    /// Get the integral term.
    pub fn get_integral_term(&self) -> T {
        self.integral_term
    }

    /// Given a new measurement and time delta, calculates the next [control output](ControlOutput).
    ///
    /// This method properly handles time intervals for mathematically accurate integral and 
    /// derivative calculations. Unlike [`next_control_output`](Self::next_control_output), this method
    /// considers the actual time elapsed between measurements.
    ///
    /// # Mathematical Accuracy
    /// 
    /// - **Integral term**: `∫e(t)dt ≈ Σ(error × dt)` - accumulates error over time
    /// - **Derivative term**: `de/dt = (measurement_change) / dt` - calculates rate of change
    ///
    /// # Arguments
    /// 
    /// * `measurement` - The current process variable measurement
    /// * `dt` - Time interval since the last measurement (must be positive)
    ///
    /// # Examples
    ///
    /// ```rust
    /// use pid::Pid;
    /// 
    /// let mut pid = Pid::new(25.0, 100.0);
    /// pid.p(1.0, 100.0).i(0.1, 100.0).d(0.01, 100.0);
    ///
    /// // Regular intervals
    /// let output1 = pid.next_control_output_with_dt(20.0, 0.1);  // 0.1 second
    /// let output2 = pid.next_control_output_with_dt(22.0, 0.1);  // 0.1 second later
    ///
    /// // Irregular intervals work correctly too
    /// let output3 = pid.next_control_output_with_dt(24.0, 0.05); // 0.05 second later
    /// let output4 = pid.next_control_output_with_dt(25.5, 0.2);  // 0.2 second later
    /// ```
    /// 
    /// # Note
    /// 
    /// If `dt` is zero or negative, the derivative term will be set to zero to avoid 
    /// division by zero or invalid calculations.
    pub fn next_control_output_with_dt(&mut self, measurement: T, dt: T) -> ControlOutput<T> {
        // Calculate the error between the ideal setpoint and the current
        // measurement to compare against
        let error = self.setpoint - measurement;

        // Calculate the proportional term and limit to it's individual limit
        let p_unbounded = error * self.kp;
        let p = apply_limit(self.p_limit, p_unbounded);

        // Modification: Taking the time interval (dt) into account in the integral term
        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        self.integral_term = self.integral_term + error * self.ki * dt;

        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.integral_term = apply_limit(self.i_limit, self.integral_term);

        // Modification: Taking the time interval (dt) into account in the differential term
        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => {
                if dt > T::zero() {
                    (measurement - *prev_measurement) / dt
                } else {
                    T::zero()  // dtが0の場合は微分項を0に
                }
            },
            None => T::zero(),
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = apply_limit(self.d_limit, d_unbounded);

        // Calculate the final output by adding together the PID terms, then
        // apply the final defined output limit
        let output = p + self.integral_term + d;
        let output = apply_limit(self.output_limit, output);

        // Return the individual term's contributions and the final output
        ControlOutput {
            p,
            i: self.integral_term,
            d,
            output,
        }
    }
}

/// Saturating the input `value` according the absolute `limit` (`-abs(limit) <= output <= abs(limit)`).
fn apply_limit<T: Number>(limit: T, value: T) -> T {
    num_traits::clamp(value, -limit.abs(), limit.abs())
}

#[cfg(test)]
mod tests {
    use super::Pid;
    use crate::ControlOutput;

    /// Proportional-only controller operation and limits
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

    /// Derivative-only controller operation and limits
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

    /// Integral-only controller operation and limits
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

    /// Checks that a full PID controller's limits work properly through multiple output iterations
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

    /// Combined PID operation
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

    // NOTE: use for new test in future: /// Full PID operation with mixed float checking to make sure they're equal
    /// PID operation with zero'd values, checking to see if different floats equal each other
    #[test]
    fn floats_zeros() {
        let mut pid_f32 = Pid::new(10.0f32, 100.0);
        pid_f32.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        let mut pid_f64 = Pid::new(10.0, 100.0f64);
        pid_f64.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        for _ in 0..5 {
            assert_eq!(
                pid_f32.next_control_output(0.0).output,
                pid_f64.next_control_output(0.0).output as f32
            );
        }
    }

    /// Full PID operation with mixed signed integer checking to make sure they're equal
    /// PID operation with zero'd values, checking to see if different floats equal each other
    #[test]
    fn signed_integers() {
        let mut pid_i8 = Pid::new(10i8, 100);
        pid_i8.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i16 = Pid::new(10i16, 100i16);
        pid_i16.p(0i16, 100i16).i(0i16, 100i16).d(0i16, 100i16);

        let mut pid_i32 = Pid::new(10i32, 100);
        pid_i32.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i64 = Pid::new(10i64, 100);
        pid_i64.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_i128 = Pid::new(10i128, 100);
        pid_i128.p(0, 100).i(0, 100).d(0, 100);

        for _ in 0..5 {
            assert_eq!(
                pid_i8.next_control_output(0i8).output as i16,
                pid_i16.next_control_output(0i16).output,
            );

            assert_eq!(
                pid_i16.next_control_output(0i16).output as i32,
                pid_i32.next_control_output(0i32).output,
            );

            assert_eq!(
                pid_i32.next_control_output(0i32).output as i64,
                pid_i64.next_control_output(0i64).output
            );

            assert_eq!(
                pid_i64.next_control_output(0i64).output as i128,
                pid_i128.next_control_output(0i128).output
            );
        }
    }

    #[test]
    fn float_match_integers() {
        let mut pid_i32 = Pid::new(10i32, 100);
        pid_i32.p(0, 100).i(0, 100).d(0, 100);

        let mut pid_f32 = Pid::new(10.0f32, 100.0);
        pid_f32.p(0.0, 100.0).i(0.0, 100.0).d(0.0, 100.0);

        for _ in 0..5 {
            assert_eq!(
                pid_i32.next_control_output(0i32).output as f32,
                pid_f32.next_control_output(0f32).output
            );
        }
    }

    /// See if the controller can properly target to the setpoint after 2 output iterations
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

    /// Make sure negative limits don't break the controller
    #[test]
    fn negative_limits() {
        let mut pid = Pid::new(10.0f32, -10.0);
        pid.p(1.0, -50.0).i(1.0, -50.0).d(1.0, -50.0);

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0);
        assert_eq!(out.i, 10.0);
        assert_eq!(out.d, 0.0);
        assert_eq!(out.output, 10.0);
    }

    /// Test that dt properly affects integral calculation
    #[test]
    fn integral_with_dt() {
        let mut pid1 = Pid::new(10.0f32, 100.0);  // Specify f32
        pid1.p(0.0, 100.0).i(1.0, 100.0).d(0.0, 100.0);
        
        let mut pid2 = Pid::new(10.0f32, 100.0);  // Specify f32
        pid2.p(0.0, 100.0).i(1.0, 100.0).d(0.0, 100.0);

        // Same error, different dt
        let output1 = pid1.next_control_output_with_dt(0.0, 0.1);  // dt = 0.1
        let output2 = pid2.next_control_output_with_dt(0.0, 0.2);  // dt = 0.2

        // Longer time interval should result in larger integral
        assert_eq!(output1.i, 1.0);  // error(10.0) * ki(1.0) * dt(0.1) = 1.0
        assert_eq!(output2.i, 2.0);  // error(10.0) * ki(1.0) * dt(0.2) = 2.0
        assert!(output2.i > output1.i);
    }

    /// Test that dt properly affects derivative calculation
    #[test]
    fn derivative_with_dt() {
        let mut pid = Pid::new(0.0f32, 100.0f32);
        pid.p(0.0f32, 100.0f32).i(0.0f32, 100.0f32).d(1.0f32, 100.0f32);

        // First measurement (no derivative yet)
        let _ = pid.next_control_output_with_dt(0.0f32, 0.1f32);

        // Second measurement with change
        let output = pid.next_control_output_with_dt(1.0f32, 0.1f32);
        
        // derivative = -(1.0 - 0.0) / 0.1 * kd(1.0) = -10.0
        assert_eq!(output.d, -10.0f32);

        // Test with different dt
        let mut pid2 = Pid::new(0.0f32, 100.0f32);
        pid2.p(0.0f32, 100.0f32).i(0.0f32, 100.0f32).d(1.0f32, 100.0f32);
        
        let _ = pid2.next_control_output_with_dt(0.0f32, 0.2f32);
        let output2 = pid2.next_control_output_with_dt(1.0f32, 0.2f32);
        
        // derivative = -(1.0 - 0.0) / 0.2 * kd(1.0) = -5.0
        assert_eq!(output2.d, -5.0f32);
        
        // Comparison without using abs() (both are negative values)
        // Since output.d = -10.0 and output2.d = -5.0
        // In absolute value, |output.d| > |output2.d|, meaning output.d < output2.d
        assert!(output.d < output2.d);
    }

    /// Test that existing method works as before (backwards compatibility)
    #[test]
    fn backwards_compatibility() {
        let mut pid_old = Pid::new(10.0f32, 100.0);  // ✅ Specify f32
        pid_old.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

        let mut pid_new = Pid::new(10.0f32, 100.0);  // ✅ Specify f32
        pid_new.p(1.0, 100.0).i(0.1, 100.0).d(1.0, 100.0);

        // Both should give same result
        let output_old = pid_old.next_control_output(5.0);
        let output_new = pid_new.next_control_output_with_dt(5.0, 1.0);  // dt = 1.0

        assert_eq!(output_old.p, output_new.p);
        assert_eq!(output_old.i, output_new.i);
        assert_eq!(output_old.d, output_new.d);
        assert_eq!(output_old.output, output_new.output);
    }

    /// Test irregular update intervals
    #[test]
    fn irregular_intervals() {
        let mut pid = Pid::new(20.0f32, 100.0);  // ✅ Specify f32
        pid.p(1.0, 100.0).i(0.5, 100.0).d(0.1, 100.0);

        // Simulate irregular sensor readings
        let _output1 = pid.next_control_output_with_dt(15.0, 0.1);   // ✅ _をつけて警告を回避
        let _output2 = pid.next_control_output_with_dt(18.0, 0.05);  // ✅ _をつけて警告を回避
        let output3 = pid.next_control_output_with_dt(17.0, 0.3);    // 実際に使用

        // Each should handle their respective dt correctly
        // Integral should accumulate: (20-15)*0.5*0.1 + (20-18)*0.5*0.05 + (20-17)*0.5*0.3
        // = 0.25 + 0.05 + 0.45 = 0.75
        assert_eq!(output3.i, 0.75);
    }
}
