import { describe, expect, it } from "vitest";

import { scaleY, TimeSeries } from "./timeSeries";

describe("TimeSeries", () => {
  it("keeps only the samples inside the window", () => {
    const series = new TimeSeries(10);
    for (let t = 0; t <= 20; t++) {
      series.push(t, [t]);
    }
    expect(series.data[0].t).toBe(10);
    expect(series.latest?.t).toBe(20);
  });

  it("drops non-finite samples", () => {
    const series = new TimeSeries(10);
    series.push(1, [Number.NaN]);
    series.push(Number.NaN, [1]);
    expect(series.data).toHaveLength(0);
  });

  it("maps values to pixel rows with clamping", () => {
    expect(scaleY(1, -1, 1, 200)).toBe(0);
    expect(scaleY(-1, -1, 1, 200)).toBe(200);
    expect(scaleY(0, -1, 1, 200)).toBe(100);
    expect(scaleY(5, -1, 1, 200)).toBe(0);
  });
});
