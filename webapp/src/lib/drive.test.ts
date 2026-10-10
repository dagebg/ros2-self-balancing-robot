import { describe, expect, it } from "vitest";

import { isDriveKey, joystickToVelocity, keysToVelocity } from "./drive";
import { DRIVE_LIMITS } from "./robot";

describe("joystickToVelocity", () => {
  it("drives forwards when pushed up and turns left (CCW) when pushed left", () => {
    expect(joystickToVelocity(0, -1)).toEqual({ linear: DRIVE_LIMITS.linear, angular: -0 });
    const left = joystickToVelocity(-1, 0);
    expect(left.angular).toBeCloseTo(DRIVE_LIMITS.angular);
    expect(left.linear).toBeCloseTo(0);
  });

  it("ignores small deflections inside the deadzone", () => {
    expect(joystickToVelocity(0.05, -0.05)).toEqual({ linear: 0, angular: 0 });
  });

  it("clamps deflections outside the unit circle", () => {
    const corner = joystickToVelocity(-1, -1);
    const magnitude = Math.hypot(
      corner.linear / DRIVE_LIMITS.linear,
      corner.angular / DRIVE_LIMITS.angular,
    );
    expect(magnitude).toBeCloseTo(1);
  });

  it("returns zero for non-finite input", () => {
    expect(joystickToVelocity(Number.NaN, 0)).toEqual({ linear: 0, angular: 0 });
  });
});

describe("keysToVelocity", () => {
  it("maps arrows and WASD, case-insensitively", () => {
    expect(keysToVelocity(["ArrowUp"]).linear).toBeCloseTo(DRIVE_LIMITS.linear);
    expect(keysToVelocity(["S"]).linear).toBeCloseTo(-DRIVE_LIMITS.linear);
    expect(keysToVelocity(["a"]).angular).toBeCloseTo(DRIVE_LIMITS.angular);
    expect(keysToVelocity(["ArrowRight"]).angular).toBeCloseTo(-DRIVE_LIMITS.angular);
  });

  it("combines keys and cancels opposites", () => {
    expect(keysToVelocity(["w", "ArrowUp"]).linear).toBeCloseTo(DRIVE_LIMITS.linear);
    expect(keysToVelocity(["w", "s"]).linear).toBeCloseTo(0);
    const curve = keysToVelocity(["w", "d"]);
    expect(curve.linear).toBeGreaterThan(0);
    expect(curve.angular).toBeLessThan(0);
  });

  it("knows which keys drive", () => {
    expect(isDriveKey("ArrowLeft")).toBe(true);
    expect(isDriveKey("W")).toBe(true);
    expect(isDriveKey("x")).toBe(false);
  });
});
