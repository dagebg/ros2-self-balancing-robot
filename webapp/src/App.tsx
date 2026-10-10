import { ArmControls } from "./components/ArmControls";
import { DrivePad } from "./components/DrivePad";
import { GainsPanel } from "./components/GainsPanel";
import { StatusPanel } from "./components/StatusPanel";
import { TelemetryChart } from "./components/TelemetryChart";
import type { RobotClient } from "./lib/client";
import { RobotState } from "./lib/robot";
import { useRobot } from "./lib/useRobot";

export function App({ client }: { client: RobotClient }) {
  const { connection, state, pose } = useRobot(client);
  const connected = connection === "connected";
  const balancing = state?.state === RobotState.Balancing;

  return (
    <main>
      <header>
        <h1>Self-balancing robot</h1>
      </header>
      <div className="grid">
        <StatusPanel connection={connection} state={state} pose={pose} />
        <ArmControls client={client} connected={connected} />
        <DrivePad client={client} enabled={connected && balancing} />
        <GainsPanel client={client} connected={connected} />
        <TelemetryChart state={state} />
      </div>
    </main>
  );
}
