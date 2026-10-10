import { describe, expect, it } from "vitest";

import {
  GAIN_PARAMETER_NAMES,
  gainsFromResponse,
  PARAMETER_DOUBLE,
  parseGain,
  setGainRequest,
} from "./gains";

describe("gains", () => {
  it("lists the bridge's nine gain parameters", () => {
    expect(GAIN_PARAMETER_NAMES).toHaveLength(9);
    expect(GAIN_PARAMETER_NAMES[0]).toBe("gains.pitch.kp");
    expect(GAIN_PARAMETER_NAMES).toContain("gains.yaw_rate.kd");
  });

  it("parses a GetParameters response in order", () => {
    const values = GAIN_PARAMETER_NAMES.map((_, i) => ({
      type: PARAMETER_DOUBLE,
      double_value: i,
    }));
    const gains = gainsFromResponse(values);
    expect(gains.pitch).toEqual({ kp: 0, ki: 1, kd: 2 });
    expect(gains.yaw_rate.kd).toBe(8);
  });

  it("rejects incomplete or non-double responses", () => {
    expect(() => gainsFromResponse([])).toThrow();
    const values = GAIN_PARAMETER_NAMES.map(() => ({ type: 2 }));
    expect(() => gainsFromResponse(values)).toThrow(/not a double/);
  });

  it("builds a SetParameters request", () => {
    expect(setGainRequest("velocity", "ki", 0.2)).toEqual({
      parameters: [
        { name: "gains.velocity.ki", value: { type: PARAMETER_DOUBLE, double_value: 0.2 } },
      ],
    });
  });

  it("accepts only finite, non-negative numbers", () => {
    expect(parseGain("28")).toBe(28);
    expect(parseGain(" 0.05 ")).toBe(0.05);
    expect(parseGain("-1")).toBeNull();
    expect(parseGain("abc")).toBeNull();
    expect(parseGain("")).toBeNull();
    expect(parseGain("Infinity")).toBeNull();
  });
});
