import { useEffect, useRef, useState } from "react";

import type { RobotClient } from "../lib/client";
import { isDriveKey, joystickToVelocity, keysToVelocity } from "../lib/drive";
import type { Velocity } from "../lib/robot";

const SEND_PERIOD_MS = 100;
const STOP: Velocity = { linear: 0, angular: 0 };

interface Props {
  client: RobotClient;
  enabled: boolean;
}

/**
 * Virtual joystick (drag inside the pad) and keyboard (arrows / WASD). While a command is active
 * it is re-sent every 100 ms; releasing sends one stop. The bridge stops the robot by itself if
 * commands cease (e.g. the tab closes).
 */
export function DrivePad({ client, enabled }: Props) {
  const pad = useRef<HTMLDivElement>(null);
  const [knob, setKnob] = useState({ x: 0, y: 0 });
  const [command, setCommand] = useState<Velocity>(STOP);
  const pressed = useRef(new Set<string>());

  // Repeat the active command; send a single stop when it returns to zero.
  useEffect(() => {
    if (!enabled) {
      return;
    }
    client.drive(command);
    if (command.linear === 0 && command.angular === 0) {
      return;
    }
    const timer = setInterval(() => client.drive(command), SEND_PERIOD_MS);
    return () => clearInterval(timer);
  }, [client, command, enabled]);

  useEffect(() => {
    const update = () => setCommand(keysToVelocity(pressed.current));
    const down = (event: KeyboardEvent) => {
      if (isDriveKey(event.key) && !(event.target instanceof HTMLInputElement)) {
        event.preventDefault();
        pressed.current.add(event.key);
        update();
      }
    };
    const up = (event: KeyboardEvent) => {
      if (pressed.current.delete(event.key)) {
        update();
      }
    };
    window.addEventListener("keydown", down);
    window.addEventListener("keyup", up);
    return () => {
      window.removeEventListener("keydown", down);
      window.removeEventListener("keyup", up);
    };
  }, []);

  const move = (event: React.PointerEvent<HTMLDivElement>) => {
    if (!pad.current || event.buttons === 0) {
      return;
    }
    const rect = pad.current.getBoundingClientRect();
    const dx = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    const dy = ((event.clientY - rect.top) / rect.height) * 2 - 1;
    const radius = Math.hypot(dx, dy);
    const scale = radius > 1 ? 1 / radius : 1;
    setKnob({ x: dx * scale, y: dy * scale });
    setCommand(joystickToVelocity(dx, dy));
  };

  const release = () => {
    setKnob({ x: 0, y: 0 });
    setCommand(STOP);
  };

  return (
    <section className="panel" aria-label="Drive">
      <h2>Drive</h2>
      <div
        ref={pad}
        className={`pad${enabled ? "" : " pad-disabled"}`}
        role="application"
        aria-label="Joystick: drag to drive"
        onPointerDown={(event) => {
          event.currentTarget.setPointerCapture(event.pointerId);
          move(event);
        }}
        onPointerMove={move}
        onPointerUp={release}
        onPointerCancel={release}
      >
        <div
          className="knob"
          style={{ left: `${50 + knob.x * 40}%`, top: `${50 + knob.y * 40}%` }}
        />
      </div>
      <p className="hint">
        {command.linear.toFixed(2)} m/s · {command.angular.toFixed(2)} rad/s — or use arrows / WASD
      </p>
    </section>
  );
}
