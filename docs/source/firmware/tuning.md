# Calibration and tuning

Work through these steps in order. Each one builds on the previous.

## 1. Measure the robot

Follow {doc}`../hardware/bringup` until every parameter in {doc}`parameters` is marked
`measured`. Wrong wiring parameters make tuning impossible: an inverted encoder, for example,
turns the velocity loop's negative feedback into positive feedback.

## 2. Check the estimator

Flash the firmware, stand the robot upright by hand, watch the telemetry (`pitch`, `pitch_rate`)
and tilt it slowly:

- Pitch must read about 0 when the robot is balanced by hand, and grow **positive when tilted
  forwards** (REP-103). If the sign is wrong, the IMU mounting parameters are wrong.
- `pitch_rate` must be close to 0 while still. If not, the robot moved during calibration; disarm,
  keep it still for 2 s and try again.

## 3. First balance

Arm the robot while holding it, bring it upright, and let go once it has started balancing
(state `balancing` after 0.5 s within 3° of upright). Keep a hand close: a wrong sign shows up as
the robot driving away from the direction it leans and falling within a second. The tilt fault
cuts the motors at 45°.

## 4. Tune one loop at a time

Gains can be changed at runtime with `SetGains` messages (see {doc}`../protocol`); persistent
defaults live in `BalanceConfig` in `shared/balance_core`. Tune from the inside out:

| Loop | Start | Increase until | Then |
|---|---|---|---|
| Pitch $K_p$ | 28 | the robot oscillates quickly (several Hz) | back off by a third |
| Pitch $K_d$ | 0.1 | oscillation is damped and it feels "stiff" | stop before motor buzz or jitter |
| Pitch $K_i$ | 5 | a constant lean offset disappears | keep small; too much causes slow rocking |
| Velocity $K_p$ | 0.3 | the robot holds its position under small pushes | back off if it starts to rock slowly |
| Velocity $K_i$ | 0.05 | it returns to rest without creeping | too much causes slow oscillation |
| Yaw rate $K_p$, $K_i$ | 0.12, 0.4 | turns follow the command | reduce if turning upsets balance |

Symptoms and causes:

| Symptom | Likely cause |
|---|---|
| Fast oscillation (> 3 Hz) | pitch $K_p$ too high or $K_d$ too low |
| Slow rocking back and forth | velocity gains too high, or pitch $K_i$ too high |
| Creeps in one direction | velocity $K_i$ too low, or the centre of mass is far off the axle |
| Falls slowly in one direction | motor or encoder inversion wrong, or motors saturating (check duty in telemetry) |
| Motors buzz | pitch $K_d$ too high for the gyro noise |

## 5. Re-check the model

With measured parameters, update the defaults of `sim::PendulumParams` in
`shared/balance_core/include/balance_core/wheeled_pendulum.hpp` and run the closed-loop tests. If
the tuned gains also work on the model, the model is trustworthy enough to try changes there
first.
