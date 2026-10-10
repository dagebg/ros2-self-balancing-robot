import { useState } from "react";

import type { RobotClient } from "../lib/client";

interface Props {
  client: RobotClient;
  connected: boolean;
}

export function ArmControls({ client, connected }: Props) {
  const [busy, setBusy] = useState(false);
  const [message, setMessage] = useState("");

  const request = async (armed: boolean) => {
    setBusy(true);
    try {
      const result = await client.arm(armed);
      setMessage(result.success ? result.message : `Failed: ${result.message}`);
    } catch (error) {
      setMessage(`Failed: ${error instanceof Error ? error.message : String(error)}`);
    } finally {
      setBusy(false);
    }
  };

  return (
    <section className="panel" aria-label="Arming">
      <h2>Arming</h2>
      <div className="buttons">
        <button type="button" disabled={!connected || busy} onClick={() => void request(true)}>
          Arm
        </button>
        <button
          type="button"
          className="danger"
          disabled={!connected || busy}
          onClick={() => void request(false)}
        >
          Disarm
        </button>
      </div>
      <p className="hint">After arming, hold the robot upright until it starts balancing.</p>
      {message && <p role="alert">{message}</p>}
    </section>
  );
}
