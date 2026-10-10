import { describe, expect, it } from "vitest";

import { faultLabels, stateName, twist, yawFromQuaternion } from "./robot";

describe("robot", () => {
  it("names states and decodes fault bits", () => {
    expect(stateName(2)).toBe("balancing");
    expect(stateName(9)).toMatch(/unknown/);
    expect(faultLabels(0)).toEqual([]);
    expect(faultLabels(0b00101)).toEqual(["tilt limit", "link timeout"]);
  });

  it("builds a Twist from a velocity", () => {
    expect(twist({ linear: 0.1, angular: -0.5 })).toEqual({
      linear: { x: 0.1, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: -0.5 },
    });
  });

  it("extracts yaw from a quaternion", () => {
    const half = Math.PI / 4;
    expect(yawFromQuaternion({ x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) })).toBeCloseTo(
      Math.PI / 2,
    );
  });
});
