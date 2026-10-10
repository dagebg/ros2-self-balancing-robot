import { useEffect, useState } from "react";

import type { ConnectionStatus, RobotClient } from "./client";
import type { BalanceState, Pose2D } from "./robot";

export interface RobotView {
  connection: ConnectionStatus;
  state: BalanceState | null;
  pose: Pose2D | null;
}

/** Connects on mount, disconnects on unmount, and exposes the latest robot state. */
export function useRobot(client: RobotClient): RobotView {
  const [connection, setConnection] = useState<ConnectionStatus>("disconnected");
  const [state, setState] = useState<BalanceState | null>(null);
  const [pose, setPose] = useState<Pose2D | null>(null);

  useEffect(() => {
    const unsubscribers = [
      client.onConnection((status) => {
        setConnection(status);
        if (status !== "connected") {
          setState(null);
        }
      }),
      client.onBalanceState(setState),
      client.onPose(setPose),
    ];
    client.connect();
    return () => {
      unsubscribers.forEach((unsubscribe) => unsubscribe());
      client.disconnect();
    };
  }, [client]);

  return { connection, state, pose };
}
