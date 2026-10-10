/** Mapping from joystick and keyboard input to velocity commands. */

import { DRIVE_LIMITS, type Velocity } from "./robot";

/**
 * Converts a joystick deflection to a velocity command.
 *
 * @param dx Horizontal deflection, -1 (left) … 1 (right).
 * @param dy Vertical deflection in screen coordinates, -1 (up) … 1 (down).
 * @param deadzone Deflections smaller than this (radius) produce no motion.
 *
 * The deflection is clamped to the unit circle; up drives forwards, left turns counter-clockwise
 * (positive yaw, REP-103).
 */
export function joystickToVelocity(
  dx: number,
  dy: number,
  deadzone = 0.1,
  limits: Velocity = DRIVE_LIMITS,
): Velocity {
  if (!Number.isFinite(dx) || !Number.isFinite(dy)) {
    return { linear: 0, angular: 0 };
  }
  const radius = Math.hypot(dx, dy);
  if (radius < deadzone) {
    return { linear: 0, angular: 0 };
  }
  const scale = radius > 1 ? 1 / radius : 1;
  const forward = -dy * scale;
  const left = -dx * scale;
  return { linear: forward * limits.linear, angular: left * limits.angular };
}

const KEY_DIRECTIONS: Record<string, { forward?: number; left?: number }> = {
  ArrowUp: { forward: 1 },
  w: { forward: 1 },
  ArrowDown: { forward: -1 },
  s: { forward: -1 },
  ArrowLeft: { left: 1 },
  a: { left: 1 },
  ArrowRight: { left: -1 },
  d: { left: -1 },
};

export function isDriveKey(key: string): boolean {
  return key in KEY_DIRECTIONS || key.toLowerCase() in KEY_DIRECTIONS;
}

/** Velocity command for the set of currently pressed keys (arrows or WASD). */
export function keysToVelocity(
  pressed: Iterable<string>,
  limits: Velocity = DRIVE_LIMITS,
): Velocity {
  let forward = 0;
  let left = 0;
  for (const key of pressed) {
    const direction = KEY_DIRECTIONS[key] ?? KEY_DIRECTIONS[key.toLowerCase()];
    forward += direction?.forward ?? 0;
    left += direction?.left ?? 0;
  }
  const clamp = (v: number) => Math.max(-1, Math.min(1, v));
  return { linear: clamp(forward) * limits.linear, angular: clamp(left) * limits.angular };
}
