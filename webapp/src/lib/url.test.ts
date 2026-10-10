import { describe, expect, it } from "vitest";

import { rosbridgeUrl } from "./url";

describe("rosbridgeUrl", () => {
  it("defaults to port 9090 on the page's host", () => {
    expect(rosbridgeUrl({ hostname: "robot.local", search: "", protocol: "http:" })).toBe(
      "ws://robot.local:9090",
    );
  });

  it("uses wss on https pages and localhost without a host", () => {
    expect(rosbridgeUrl({ hostname: "", search: "", protocol: "https:" })).toBe(
      "wss://localhost:9090",
    );
  });

  it("can be overridden with ?ros=", () => {
    expect(
      rosbridgeUrl({ hostname: "x", search: "?ros=ws://10.42.0.100:9090", protocol: "http:" }),
    ).toBe("ws://10.42.0.100:9090");
  });
});
