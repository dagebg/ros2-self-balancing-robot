# ADR-0009: Balance with cascaded PID loops rather than a state-space (LQR) controller

- **Status:** Accepted
- **Date:** 2026-10-10

## Context

The robot must stay upright, follow velocity and yaw-rate commands, and be tunable on the real
hardware. Its physical parameters (mass, centre-of-mass height, inertia, motor constants) are not
measured yet, and the motor model is only approximate.

## Decision

Use three PID loops: velocity → lean angle → motor command, plus a yaw-rate loop mixed
differentially. The pitch loop's D term uses the gyroscope rate. The default gains are chosen for
robustness across a range of plant parameters with the closed-loop model.

## Alternatives considered

- **LQR on the linearized model:** optimal for the model and handles the coupling between position
  and pitch directly. It depends on accurate parameters, though, which do not exist yet, and its
  gain matrix is hard to adjust intuitively on the robot. It remains a candidate once the
  parameters are measured (see the roadmap).
- **Single PID on pitch only:** balances but drifts, because a constant lean offset turns into
  constant acceleration; it cannot follow velocity commands.

## Consequences

- Each loop has a physical meaning (how hard to correct a tilt, how far to lean for a speed error)
  and can be tuned one at a time, live from the web app.
- The cascade only works if the inner loop is clearly faster than the outer one; the gain limits
  and the 8° lean limit keep the outer loop gentle.
- The plant model and closed-loop tests make it cheap to compare an LQR controller later with the
  same tests.
