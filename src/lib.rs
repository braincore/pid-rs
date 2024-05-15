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
//! let mut pid = Pid::new()
//!     .setpoint(15.0)
//!     .clamp(-100.0, 100.0)
//!     .p(10.0);
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
//! pid.i(1.0);
//! let output = pid.next_control_output(10.0);
//!
//! // Now that the integral makes the controller stateful, it will change
//! assert_eq!(output.output, 55.0); // <--
//! assert_eq!(output.p, 50.0);
//! assert_eq!(output.i, 5.0);
//!
//! // Add our final derivative term and match our setpoint target
//! pid.d(2.0);
//! let output = pid.next_control_output(15.0);
//!
//! // The output will now say to go down due to the derivative
//! assert_eq!(output.output, -5.0); // <--
//! assert_eq!(output.p, 0.0);
//! assert_eq!(output.i, 5.0);
//! assert_eq!(output.d, -10.0);
//! ```
#![no_std]

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
pub trait Number: num_traits::sign::Signed + PartialOrd + Default + Copy + Send {
    fn clamp(self, min: Self, max: Self) -> Self;
}

// Implement `Number` for all types that
// satisfy `PartialOrd + num_traits::Signed + Copy`.
impl<T> Number for T
where
    T: num_traits::sign::Signed
        + PartialOrd
        + Default
        + Copy
        + Send,
{
    fn clamp(self, min: Self, max: Self) -> Self {
        if self < min {min} else 
        if self > max {max} else 
        {self}
    }
}

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
/// let mut p_controller = Pid::new()
///     .setpoint(15.0)
///     .clamp(-100.0, 100.0)
///     .p(10.0);
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
/// let mut full_controller = Pid::new()
///     .setpoint(15.0)
///     .clamp(-100.0, 100.0);
///     .p(10.0)
///     .i(4.5)
///     .d(0.25);
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
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
pub struct Pid<T: Number> {
    /// Ideal setpoint to strive for.
    pub setpoint: T,
    /// Proportional gain.
    pub kp: T,
    /// Integral gain.
    pub ki: T,
    /// Derivative gain.
    pub kd: T,
    /// Limiter for the proportional term: `-p_limit <= P <= p_limit`.
    pub p_limit_high: T,
    /// Limiter for the derivative term: `p_limit_low <= P <= p_limit`.
    pub p_limit_low: T,
    /// Limiter for the integral term: `i_limit_low <= I <= i_limit_high`.
    pub i_limit_high: T,
    /// Limiter for the derivative term: `i_limit_low <= I <= i_limit_high`.
    pub i_limit_low: T,
    /// Limiter for the derivative term: `d_limit_low <= D <= d_limit_high`.
    pub d_limit_high: T,
    /// Limiter for the derivative term: `d_limit_low <= D <= d_limit_high`.
    pub d_limit_low: T,
    /// Limiter for the derivative term: `o_limit_low <= O <= o_limit_high`.
    pub o_limit_high: T,
    /// Limiter for the derivative term: `o_limit_low <= O <= o_limit_high`.
    pub o_limit_low: T,
    /// Last calculated integral value if [Pid::ki] is used.
    pub i_term: T,
    /// Previously found measurement whilst using the [Pid::next_control_output] method.
    pub prev_measurement: Option<T>,
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
/// let mut pid = Pid::new()
///     .setpoint(15.0)
///     .clamp(0.0, 100.0)
///     .p(10.0)
///     .i(1.0)
///     .d(2.0);
///
/// // Input an example value and get a report for an output iteration
/// let output = pid.next_control_output(26.2456);
/// println!("P: {}\nI: {}\nD: {}\nFinal Output: {}", output.p, output.i, output.d, output.output);
/// ```
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
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

impl const<T: Number, F: Number> From<Pid<F>> for Pid<T>
where
    T: From<F>
{
    fn from(value: Pid<F>) -> Self {
        Self {
            setpoint: T::from(value.setpoint),
            kp: T::from(value.kp),
            ki: T::from(value.ki),
            kd: T::from(value.kd),
            p_limit_high: T::from(value.p_limit_high),
            p_limit_low: T::from(value.p_limit_low),
            i_limit_high: T::from(value.i_limit_high),
            i_limit_low: T::from(value.i_limit_low),
            d_limit_high: T::from(value.d_limit_high),
            d_limit_low: T::from(value.d_limit_low),
            o_limit_high: T::from(value.o_limit_high),
            o_limit_low: T::from(value.o_limit_low),
            i_term: T::from(value.i_term),
            prev_measurement: value.prev_measurement.map(|v| T::from(v)),
        }
    }
}

impl Pid<i8>
{
    /// Creates a new controller
    ///
    /// To set your P, I, and D gains into this controller, please use the following builder methods:
    /// - [Self::p()]: Proportional gain setting
    /// - [Self::i()]: Integral gain setting
    /// - [Self::d()]: Derivative gain setting
    pub const fn new() -> Self {
        Self {
            setpoint: 0,
            kp: 0,
            ki: 0,
            kd: 0,
            p_limit_high: 0,
            p_limit_low: 0,
            i_limit_high: 0,
            i_limit_low: 0,
            d_limit_high: 0,
            d_limit_low: 0,
            o_limit_high: 0,
            o_limit_low: 0,
            i_term: 0,
            prev_measurement: None,
        }
    }
}

impl Pid<i16>
{
    pub const fn new() -> Self {
        Self { ..Pid::<i8>::new().into() }
    }
}

impl Pid<i32>
{
    pub const fn new() -> Self {
        Self { ..Pid::<i8>::new().into() }
    }
}

impl Pid<i64>
{
    pub const fn new() -> Self {
        Self { ..Pid::<i8>::new().into() }
    }
}

impl Pid<i128>
{
    pub const fn new() -> Self {
        Self { ..Pid::<i8>::new().into() }
    }
}

impl Pid<f32>
{
    pub const fn new() -> Self {
        Self {
            setpoint: 0.0,
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
            p_limit_high: 0.0,
            p_limit_low: 0.0,
            i_limit_high: 0.0,
            i_limit_low: 0.0,
            d_limit_high: 0.0,
            d_limit_low: 0.0,
            o_limit_high: 0.0,
            o_limit_low: 0.0,
            i_term: 0.0,
            prev_measurement: None,
        }
    }
}

impl Pid<f64>
{
    pub const fn new() -> Self {
        Self { ..Pid::<f32>::new().into() }
    }
}

impl<T: Number> Pid<T>
{
    /// Sets the [Self::p] gain for this controller.
    pub fn p(&mut self, gain: impl Into<T>) -> &mut Self {
        self.kp = gain.into();
        self
    }

    /// Sets the [Self::i] gain for this controller.
    pub fn i(&mut self, gain: impl Into<T>) -> &mut Self {
        self.ki = gain.into();
        self
    }

    /// Sets the [Self::d] gain for this controller.
    pub fn d(&mut self, gain: impl Into<T>) -> &mut Self {
        self.kd = gain.into();
        self
    }

    /// Sets the [Pid::setpoint] to target for this controller.
    pub fn setpoint(&mut self, setpoint: impl Into<T>) -> &mut Self {
        self.setpoint = setpoint.into();
        self
    }

    /// Sets the [Self::p] limit for this controller.
    pub fn clamp(&mut self, low: impl Into<T>, high: impl Into<T>) -> &mut Self {
        let low: T = low.into();
        let high: T = high.into();
        self.p_limit_low = low;
        self.p_limit_high = high;
        self.i_limit_low = low;
        self.i_limit_high = high;
        self.d_limit_low = low;
        self.d_limit_high = high;
        self.o_limit_low = low;
        self.o_limit_high = high;
        self
    }

    /// Set integral term to a custom value. This might be useful to restore the
    /// pid controller to a previous state after an interruption or crash.
    pub fn set_integral_term(&mut self, i_term: impl Into<T>) -> &mut Self {
        let i_unbound: T = i_term.into();
        self.i_term = i_unbound.clamp(self.i_limit_low, self.i_limit_high);
        self
    }

    /// Resets the integral term back to zero, this may drastically change the
    /// control output.
    pub fn reset_integral_term(&mut self) -> &mut Self {
        self.set_integral_term(T::default())
    }

    /// Given a new measurement, calculates the next [control output](ControlOutput).
    ///
    /// # Panics
    ///
    /// - If a setpoint has not been set via `update_setpoint()`.
    pub fn next_control_output(&mut self, measurement: impl Into<T>) -> ControlOutput<T> {
        let measurement: T = measurement.into();
        
        // Calculate the error between the ideal setpoint and the current
        // measurement to compare against
        let error = self.setpoint - measurement;

        // Calculate the proportional term and limit to it's individual limit
        let p_unbounded = error * self.kp;
        let p = p_unbounded.clamp(self.p_limit_low, self.p_limit_high);

        // Mitigate output jumps when ki(t) != ki(t-1).
        // While it's standard to use an error_integral that's a running sum of
        // just the error (no ki), because we support ki changing dynamically,
        // we store the entire term so that we don't need to remember previous
        // ki values.
        let i_unbounded = self.i_term + error * self.ki;

        // Mitigate integral windup: Don't want to keep building up error
        // beyond what i_limit will allow.
        self.i_term = i_unbounded.clamp(self.i_limit_low, self.i_limit_high);

        // Mitigate derivative kick: Use the derivative of the measurement
        // rather than the derivative of the error.
        let d_unbounded = -match self.prev_measurement.as_ref() {
            Some(prev_measurement) => measurement - *prev_measurement,
            None => T::default(),
        } * self.kd;
        self.prev_measurement = Some(measurement);
        let d = d_unbounded.clamp(self.d_limit_low, self.d_limit_high);

        // Calculate the final output by adding together the PID terms, then
        // apply the final defined output limit
        let o_unbounded = p + self.i_term + d;
        let output = o_unbounded.clamp(self.o_limit_low, self.o_limit_high);

        // Return the individual term's contributions and the final output
        ControlOutput {
            p,
            i: self.i_term,
            d,
            output,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Pid;
    use crate::ControlOutput;

    /// Proportional-only controller operation and limits
    #[test]
    fn proportional() {
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .p(2.0);
        
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
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .d(2.0);

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
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .i(2.0);

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
        let mut pid2 = Pid::new()
            .setpoint(-10.0)
            .clamp(-100.0, 100.0)
            .i(2.0);

        assert_eq!(pid2.next_control_output(0.0).output, -20.0);
        assert_eq!(pid2.next_control_output(0.0).output, -40.0);

        pid2.i_limit_high = 50.0;
        pid2.i_limit_low = -50.0;
        assert_eq!(pid2.next_control_output(-5.0).output, -50.0);
        // Test that limit doesn't impede reversal of error integral
        assert_eq!(pid2.next_control_output(-15.0).output, -40.0);
    }

    /// Checks that a full PID controller's limits work properly through multiple output iterations
    #[test]
    fn output_limit() {
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .p(1.0);

        pid.o_limit_high = 1.0;
        pid.o_limit_low = -1.0;

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
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .p(1.0)
            .i(0.1)
            .d(1.0);

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
        let mut pid_f32 = Pid::new()
            .setpoint(10.0f32)
            .clamp(-100.0, 100.0);

        let mut pid_f64 = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0f64, 100.0f64);

        for _ in 0..5 {
            assert_eq!(
                pid_f32.next_control_output(0.0).output,
                pid_f64.next_control_output(0.0).output as f32
            );
        }
    }

    // NOTE: use for new test in future: /// Full PID operation with mixed signed integer checking to make sure they're equal
    /// PID operation with zero'd values, checking to see if different floats equal each other
    #[test]
    fn signed_integers_zeros() {
        let mut pid_i8 = Pid::new()
            .setpoint(10i8)
            .clamp(-100, 100);

        let mut pid_i32 = Pid::new()
            .setpoint(10i32)
            .clamp(-100, 100);

        for _ in 0..5 {
            assert_eq!(
                pid_i32.next_control_output(0).output,
                pid_i8.next_control_output(0i8).output as i32
            );
        }
    }

    /// See if the controller can properly target to the setpoint after 2 output iterations
    #[test]
    fn setpoint() {
        let mut pid = Pid::new()
            .setpoint(10.0)
            .clamp(-100.0, 100.0)
            .p(1.0)
            .i(0.1)
            .d(1.0);

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
        let mut pid = Pid::new()
            .setpoint(10.0f32)
            .clamp(50.0, -50.0)
            .p(1.0)
            .i(1.0)
            .d(1.0);

        pid.o_limit_high = -10.0;
        pid.o_limit_low = 10.0;

        let out = pid.next_control_output(0.0);
        assert_eq!(out.p, 10.0);
        assert_eq!(out.i, 10.0);
        assert_eq!(out.d, 0.0);
        assert_eq!(out.output, 10.0);
    }
}
