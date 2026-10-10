import type { CommandResult, ConnectionStatus, RobotClient } from "../lib/client";
import type { Gains, Loop, Term } from "../lib/gains";
import type { BalanceState, Pose2D, Velocity } from "../lib/robot";

type Listener<T> = (value: T) => void;

/** In-memory RobotClient: tests drive its events and inspect what the app sent. */
export class FakeClient implements RobotClient {
  readonly driven: Velocity[] = [];
  readonly armRequests: boolean[] = [];
  readonly gainUpdates: Array<{ loop: Loop; term: Term; value: number }> = [];
  armResult: CommandResult = { success: true, message: "armed" };
  gainResult: CommandResult = { success: true, message: "" };
  gains: Gains = {
    pitch: { kp: 28, ki: 5, kd: 0.1 },
    velocity: { kp: 0.3, ki: 0.05, kd: 0 },
    yaw_rate: { kp: 0.12, ki: 0.4, kd: 0 },
  };
  connected = false;

  private readonly connection = new Set<Listener<ConnectionStatus>>();
  private readonly states = new Set<Listener<BalanceState>>();
  private readonly poses = new Set<Listener<Pose2D>>();

  connect(): void {
    this.connected = true;
    this.connection.forEach((l) => l("connected"));
  }

  disconnect(): void {
    this.connected = false;
  }

  onConnection(listener: Listener<ConnectionStatus>) {
    this.connection.add(listener);
    return () => this.connection.delete(listener);
  }

  onBalanceState(listener: Listener<BalanceState>) {
    this.states.add(listener);
    return () => this.states.delete(listener);
  }

  onPose(listener: Listener<Pose2D>) {
    this.poses.add(listener);
    return () => this.poses.delete(listener);
  }

  drive(velocity: Velocity): void {
    this.driven.push(velocity);
  }

  arm(armed: boolean): Promise<CommandResult> {
    this.armRequests.push(armed);
    return Promise.resolve(this.armResult);
  }

  getGains(): Promise<Gains> {
    return Promise.resolve(structuredClone(this.gains));
  }

  setGain(loop: Loop, term: Term, value: number): Promise<CommandResult> {
    this.gainUpdates.push({ loop, term, value });
    return Promise.resolve(this.gainResult);
  }

  emitState(state: Partial<BalanceState>): void {
    const full: BalanceState = {
      state: 0,
      faults: 0,
      roll: 0,
      pitch: 0,
      pitch_rate: 0,
      yaw_rate: 0,
      left_command: 0,
      right_command: 0,
      max_loop_time_us: 0,
      ...state,
    };
    this.states.forEach((l) => l(full));
  }

  emitConnection(status: ConnectionStatus): void {
    this.connection.forEach((l) => l(status));
  }
}
