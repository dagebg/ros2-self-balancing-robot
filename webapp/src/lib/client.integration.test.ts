// @vitest-environment node
/**
 * Integration test of RosbridgeClient against a running robot or simulation:
 *
 *   ROSBRIDGE_URL=ws://localhost:9090 npm run test:integration
 *
 * Skipped unless ROSBRIDGE_URL is set. Expects the robot to be balancing (the simulation arms
 * itself after start-up).
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";

import { RosbridgeClient } from "./client";
import type { BalanceState } from "./robot";
import { RobotState } from "./robot";

const url = process.env.ROSBRIDGE_URL;

function waitFor<T>(register: (resolve: (value: T) => void) => void, ms = 15000): Promise<T> {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error("timeout")), ms);
    register((value) => {
      clearTimeout(timer);
      resolve(value);
    });
  });
}

describe.skipIf(!url)("RosbridgeClient against rosbridge", () => {
  const client = new RosbridgeClient(url ?? "");

  beforeAll(async () => {
    const connected = waitFor<void>((resolve) =>
      client.onConnection((status) => status === "connected" && resolve()),
    );
    client.connect();
    await connected;
  });

  afterAll(() => client.disconnect());

  it("receives balance state", async () => {
    const state = await waitFor<BalanceState>((resolve) => client.onBalanceState(resolve));
    expect(state.state).toBeGreaterThanOrEqual(RobotState.Disarmed);
  });

  it("arms through the bridge (idempotent while balancing)", async () => {
    const result = await client.arm(true);
    expect(result.success).toBe(true);
  });

  it("reads and writes gains", async () => {
    const gains = await client.getGains();
    expect(gains.pitch.kp).toBeGreaterThan(0);
    const result = await client.setGain("velocity", "ki", gains.velocity.ki);
    expect(result.success).toBe(true);
    const rejected = await client.setGain("pitch", "kd", -1);
    expect(rejected.success).toBe(false);
  });

  it("drives the robot forwards", async () => {
    const start = await waitFor<{ x: number }>((resolve) => client.onPose(resolve));
    const timer = setInterval(() => client.drive({ linear: 0.1, angular: 0 }), 100);
    await new Promise((resolve) => setTimeout(resolve, 4000));
    clearInterval(timer);
    client.drive({ linear: 0, angular: 0 });
    const end = await waitFor<{ x: number }>((resolve) => client.onPose(resolve));
    expect(end.x - start.x).toBeGreaterThan(0.1);
  }, 20000);
});
