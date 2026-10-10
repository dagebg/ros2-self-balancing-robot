import { act, fireEvent, render, screen, waitFor } from "@testing-library/react";
import { describe, expect, it } from "vitest";

import { App } from "./App";
import { FakeClient } from "./test/fakeClient";

function renderApp() {
  const client = new FakeClient();
  render(<App client={client} />);
  return client;
}

describe("App", () => {
  it("connects on start and shows the robot state with faults", async () => {
    const client = renderApp();
    expect(client.connected).toBe(true);
    expect(screen.getByRole("status")).toHaveTextContent("connected");

    act(() => client.emitState({ state: 3, faults: 0b1, pitch: 0.5 }));
    expect(screen.getByText("fault")).toBeInTheDocument();
    expect(screen.getByText("tilt limit")).toBeInTheDocument();
    expect(screen.getByText("28.6°")).toBeInTheDocument();
    await waitFor(() => expect(screen.getByLabelText("Pitch kp")).toHaveValue("28"));
  });

  it("arms and disarms through the client and shows the answer", async () => {
    const client = renderApp();
    fireEvent.click(screen.getByRole("button", { name: "Arm" }));
    await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent("armed"));

    client.armResult = { success: false, message: "rejected: fault" };
    fireEvent.click(screen.getByRole("button", { name: "Disarm" }));
    await waitFor(() =>
      expect(screen.getByRole("alert")).toHaveTextContent("Failed: rejected: fault"),
    );
    expect(client.armRequests).toEqual([true, false]);
  });

  it("disables arming while disconnected", () => {
    const client = renderApp();
    act(() => client.emitConnection("disconnected"));
    expect(screen.getByRole("button", { name: "Arm" })).toBeDisabled();
  });

  it("sends gain changes and rejects invalid input locally", async () => {
    const client = renderApp();
    const input = await screen.findByLabelText("Velocity ki");

    fireEvent.change(input, { target: { value: "0.2" } });
    fireEvent.keyDown(input, { key: "Enter" });
    await waitFor(() =>
      expect(client.gainUpdates).toEqual([{ loop: "velocity", term: "ki", value: 0.2 }]),
    );

    fireEvent.change(input, { target: { value: "-3" } });
    fireEvent.blur(input);
    await waitFor(() => expect(screen.getByRole("alert")).toHaveTextContent("non-negative"));
    expect(client.gainUpdates).toHaveLength(1);
  });

  it("drives with the keyboard only while balancing", () => {
    const client = renderApp();
    act(() => client.emitState({ state: 1 }));
    fireEvent.keyDown(window, { key: "ArrowUp" });
    expect(client.driven).toHaveLength(0);
    fireEvent.keyUp(window, { key: "ArrowUp" });

    act(() => client.emitState({ state: 2 }));
    fireEvent.keyDown(window, { key: "ArrowUp" });
    expect(client.driven.at(-1)?.linear).toBeGreaterThan(0);
    fireEvent.keyUp(window, { key: "ArrowUp" });
    expect(client.driven.at(-1)).toEqual({ linear: 0, angular: 0 });
  });
});
