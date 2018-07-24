# PID Controller for Rust

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

## Assumptions

* All calculations use `f32`.
* Measurements occur at equal spacing. (`t(i) = t(i-1) + C`)
* Output limits per term are symmetric around 0 (`-limit < term < limit`).

## Todo

- [ ] Three-term output limit: `MAX(p + i - d, global_limit)`.
- [ ] Support `f64` via generic type parameters.
- [ ] Helper for (auto-)tuning by detecting frequency & amplitude of
      oscillations.
