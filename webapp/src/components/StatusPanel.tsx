import type { ConnectionStatus } from "../lib/client";
import { faultLabels, RobotState, stateName } from "../lib/robot";
import type { BalanceState, Pose2D } from "../lib/robot";

const toDegrees = (radians: number) => (radians * 180) / Math.PI;

interface Props {
  connection: ConnectionStatus;
  state: BalanceState | null;
  pose: Pose2D | null;
}

export function StatusPanel({ connection, state, pose }: Props) {
  const faults = state ? faultLabels(state.faults) : [];
  const tone =
    state?.state === RobotState.Fault
      ? "bad"
      : state?.state === RobotState.Balancing
        ? "good"
        : "neutral";

  return (
    <section className="panel" aria-label="Status">
      <h2>Status</h2>
      <p>
        <span className={`badge badge-${connection}`} role="status">
          {connection}
        </span>
      </p>
      <dl className="readout">
        <dt>State</dt>
        <dd className={`tone-${tone}`}>{state ? stateName(state.state) : "–"}</dd>
        <dt>Pitch</dt>
        <dd>{state ? `${toDegrees(state.pitch).toFixed(1)}°` : "–"}</dd>
        <dt>Motors</dt>
        <dd>
          {state ? `${state.left_command.toFixed(2)} / ${state.right_command.toFixed(2)}` : "–"}
        </dd>
        <dt>Loop time</dt>
        <dd>{state ? `${state.max_loop_time_us} µs` : "–"}</dd>
        <dt>Position</dt>
        <dd>
          {pose
            ? `${pose.x.toFixed(2)} m, ${pose.y.toFixed(2)} m, ${toDegrees(pose.heading).toFixed(0)}°`
            : "–"}
        </dd>
      </dl>
      {faults.length > 0 && (
        <ul className="faults" aria-label="Faults">
          {faults.map((fault) => (
            <li key={fault}>{fault}</li>
          ))}
        </ul>
      )}
    </section>
  );
}
