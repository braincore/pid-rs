# PID Controller for Rust [![Latest Version]][crates.io] [![Documentation]][docs.rs] [![Build Status]][travis] 

[Build Status]: https://api.travis-ci.org/braincore/pid-rs.svg?branch=master
[travis]: https://travis-ci.org/braincore/pid-rs
[Latest Version]: https://img.shields.io/crates/v/pid.svg
[crates.io]: https://crates.io/crates/pid
[Documentation]: https://docs.rs/pid/badge.svg
[docs.rs]: https://docs.rs/pid

A proportional-integral-derivative (PID) controller.

## Features

* Visibility into individual contribution of P, I, and D terms which often
  need to be logged for later analysis and parameter tuning.
* Output limits on a per term basis.
* Mitigation of integral windup using integral term limit.
* Mitigation of derivative kick by using the derivative of the measurement
  rather than the derivative of the error.
* On-the-fly changes to `setpoint`/`kp`/`ki`/`kd`.
  * Mitigation of output jumps when changing `ki` by storing the integration of
    `e(t) * ki(t)` rather than only `e(t)`.
* Generic float type parameter to support `f32` or `f64`.
* Support for `no_std` environments, such as embedded systems.
* Optional support for [Serde](https://crates.io/crates/serde). Enable the
  `serde` Cargo feature, if you need `Pid` to implement
  `Serialize`/`Deserialize`.

## Example

```rust
extern crate pid;
use pid::Pid;

fn main() {
    // Set only kp (proportional) to 10. The setpoint is 15.
    // Set limits for P, I, and D to 100 each.
    let mut pid = Pid::new(10.0, 0.0, 0.0, 100.0, 100.0, 100.0, 15.0);
    // Fake a measurement of 10.0, which is an error of 5.0.
    let output = pid.next_control_output(10.0);
    // Verify that kp * error = 10.0 * 5.0 = 50.0
    assert_eq!(output.output, 50.0);
    // Verify that all output was from the proportional term
    assert_eq!(output.p, 50.0);
    assert_eq!(output.i, 0.0);
    assert_eq!(output.d, 0.0);
    
    // Verify that the same measurement produces the same output since we
    // aren't using the stateful derivative & integral terms.
    let output = pid.next_control_output(10.0);
    assert_eq!(output.p, 50.0);
    
    // Add an integral term
    pid.ki = 1.0;
    let output = pid.next_control_output(10.0);
    assert_eq!(output.p, 50.0);
    // Verify that the integral term is adding to the output signal.
    assert_eq!(output.i, 5.0);
    assert_eq!(output.output, 55.0);

    // Add a derivative term
    pid.kd = 2.0;
    let output = pid.next_control_output(15.0);  // Match the desired target
    // No proportional term since no error
    assert_eq!(output.p, 0.0);
    // Integral term stays the same
    assert_eq!(output.i, 5.0);
    // Derivative on measurement produces opposing signal
    assert_eq!(output.d, -10.0);
    assert_eq!(output.output, -5.0);
}
```

## Assumptions

* Measurements occur at equal spacing. (`t(i) = t(i-1) + C`)
* Output limits per term are symmetric around 0 (`-limit <= term <= limit`).

## Formulation

There are several different formulations of PID controllers. This library
uses the independent form:

![PID independent form](
https://latex.codecogs.com/gif.latex?C(t)&space;=&space;&space;K_p&space;\cdot&space;e(t)&space;&plus;&space;K_i&space;\cdot&space;\int{e(t)dt}&space;-&space;K_d&space;\cdot&space;\frac{dP(t)}{dt})

where:
- C(t) = control output, the output to the actuator.
- P(t) = process variable, the measured value.
- e(t) = error = S(t) - P(t)
- S(t) = set point, the desired target for the process variable.

`kp`/`ki`/`kd` can be changed during operation and can therefore be a function
of time.

If you're interested in the dependent form, add your own logic that computes
`kp`/`ki`/`kd` using dead time, time constant, `kc`, or whatever else.

## Todo

- [ ] Three-term output limit: `MAX(p + i + d, global_limit)`.
- [ ] Helper for (auto-)tuning by detecting frequency & amplitude of
      oscillations.
