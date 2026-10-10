/**
 * Robot-facing types and constants, mirroring robot_interfaces/BalanceState and the bridge's
 * interfaces (see docs: ROS 2 > robot_bridge).
 */

export const RobotState = {
  Disarmed: 0,
  Armed: 1,
  Balancing: 2,
  Fault: 3,
} as const;
export type RobotStateValue = (typeof RobotState)[keyof typeof RobotState];

export const STATE_NAMES: Record<number, string> = {
  [RobotState.Disarmed]: "disarmed",
  [RobotState.Armed]: "armed, waiting to be upright",
  [RobotState.Balancing]: "balancing",
  [RobotState.Fault]: "fault",
};

/** Fault bits of BalanceState.faults (and the firmware protocol). */
export const FAULTS: ReadonlyArray<{ bit: number; label: string }> = [
  { bit: 1 << 0, label: "tilt limit" },
  { bit: 1 << 1, label: "IMU failure" },
  { bit: 1 << 2, label: "link timeout" },
  { bit: 1 << 3, label: "IMU disagreement" },
  { bit: 1 << 4, label: "loop overrun" },
];

export function stateName(state: number): string {
  return STATE_NAMES[state] ?? `unknown (${state})`;
}

export function faultLabels(faults: number): string[] {
  return FAULTS.filter(({ bit }) => (faults & bit) !== 0).map(({ label }) => label);
}

/** robot_interfaces/msg/BalanceState as delivered by rosbridge. */
export interface BalanceState {
  state: number;
  faults: number;
  roll: number;
  pitch: number;
  pitch_rate: number;
  yaw_rate: number;
  left_command: number;
  right_command: number;
  max_loop_time_us: number;
}

export interface Pose2D {
  x: number;
  y: number;
  heading: number;
}

export interface Velocity {
  linear: number;
  angular: number;
}

export interface Twist {
  linear: { x: number; y: number; z: number };
  angular: { x: number; y: number; z: number };
}

export function twist(velocity: Velocity): Twist {
  return {
    linear: { x: velocity.linear, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: velocity.angular },
  };
}

/** Speed limits for manual driving; well inside the controller's own limits. */
export const DRIVE_LIMITS: Velocity = { linear: 0.15, angular: 1.2 };

/** Yaw from a quaternion (rotation about z). */
export function yawFromQuaternion(q: { x: number; y: number; z: number; w: number }): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
