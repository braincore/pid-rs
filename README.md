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
* On-the-fly changes to `kp`/`ki`/`kd`.
  * Mitigation of output jumps when changing `ki` by storing the integration of
    `e(t) * ki(t)` rather than only `e(t)`.
* Generic float type parameter to support `f32` or `f64`.
* Support for `no_std` environments, such as embedded systems.

## Assumptions

* Measurements occur at equal spacing. (`t(i) = t(i-1) + C`)
* Output limits per term are symmetric around 0 (`-limit <= term <= limit`).

## Todo

- [ ] Three-term output limit: `MAX(p + i + d, global_limit)`.
- [ ] Helper for (auto-)tuning by detecting frequency & amplitude of
      oscillations.
