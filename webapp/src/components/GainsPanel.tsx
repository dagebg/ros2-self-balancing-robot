import { useEffect, useState } from "react";

import type { RobotClient } from "../lib/client";
import { LOOP_LABELS, LOOPS, parseGain, TERMS } from "../lib/gains";
import type { Gains, Loop, Term } from "../lib/gains";

interface Props {
  client: RobotClient;
  connected: boolean;
}

type Drafts = Partial<Record<string, string>>;

/** Live PID tuning: values are ROS parameters of /robot_bridge, which forwards them to the ESP32. */
export function GainsPanel({ client, connected }: Props) {
  const [gains, setGains] = useState<Gains | null>(null);
  const [drafts, setDrafts] = useState<Drafts>({});
  const [message, setMessage] = useState("");

  useEffect(() => {
    if (!connected) {
      return;
    }
    let cancelled = false;
    client
      .getGains()
      .then((loaded) => {
        if (!cancelled) {
          setGains(loaded);
          setDrafts({});
          setMessage("");
        }
      })
      .catch((error: unknown) => {
        if (!cancelled) {
          setMessage(
            `Could not read gains: ${error instanceof Error ? error.message : String(error)}`,
          );
        }
      });
    return () => {
      cancelled = true;
    };
  }, [client, connected]);

  const apply = async (loop: Loop, term: Term) => {
    const key = `${loop}.${term}`;
    const text = drafts[key];
    if (text === undefined) {
      return;
    }
    const value = parseGain(text);
    if (value === null) {
      setMessage(`${LOOP_LABELS[loop]} ${term}: enter a non-negative number`);
      return;
    }
    const result = await client.setGain(loop, term, value).catch((error: unknown) => ({
      success: false,
      message: error instanceof Error ? error.message : String(error),
    }));
    if (result.success) {
      setGains((current) =>
        current ? { ...current, [loop]: { ...current[loop], [term]: value } } : current,
      );
      setDrafts(({ [key]: _, ...rest }) => rest);
      setMessage(`${LOOP_LABELS[loop]} ${term} = ${value}`);
    } else {
      setMessage(`Rejected: ${result.message}`);
    }
  };

  return (
    <section className="panel" aria-label="Controller gains">
      <h2>Controller gains</h2>
      {gains ? (
        <table className="gains">
          <thead>
            <tr>
              <th scope="col">Loop</th>
              {TERMS.map((term) => (
                <th scope="col" key={term}>
                  {term}
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {LOOPS.map((loop) => (
              <tr key={loop}>
                <th scope="row">{LOOP_LABELS[loop]}</th>
                {TERMS.map((term) => {
                  const key = `${loop}.${term}`;
                  return (
                    <td key={term}>
                      <input
                        aria-label={`${LOOP_LABELS[loop]} ${term}`}
                        inputMode="decimal"
                        value={drafts[key] ?? String(gains[loop][term])}
                        onChange={(event) =>
                          setDrafts((d) => ({ ...d, [key]: event.target.value }))
                        }
                        onBlur={() => void apply(loop, term)}
                        onKeyDown={(event) => {
                          if (event.key === "Enter") {
                            void apply(loop, term);
                          }
                        }}
                      />
                    </td>
                  );
                })}
              </tr>
            ))}
          </tbody>
        </table>
      ) : (
        <p className="hint">{connected ? "Loading…" : "Connect to read the gains."}</p>
      )}
      {message && <p role="alert">{message}</p>}
    </section>
  );
}
