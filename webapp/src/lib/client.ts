/**
 * The web app's view of the robot. Components depend only on this interface; RosbridgeClient is
 * the real implementation and tests use a fake.
 */

import { Ros, Service, Topic } from "roslib";

import { GAIN_PARAMETER_NAMES, gainsFromResponse, setGainRequest } from "./gains";
import type { Gains, Loop, Term } from "./gains";
import { twist, yawFromQuaternion } from "./robot";
import type { BalanceState, Pose2D, Twist, Velocity } from "./robot";

export type ConnectionStatus = "connecting" | "connected" | "disconnected";

export interface CommandResult {
  success: boolean;
  message: string;
}

export interface RobotClient {
  connect(): void;
  disconnect(): void;
  onConnection(listener: (status: ConnectionStatus) => void): () => void;
  onBalanceState(listener: (state: BalanceState) => void): () => void;
  onPose(listener: (pose: Pose2D) => void): () => void;
  drive(velocity: Velocity): void;
  arm(armed: boolean): Promise<CommandResult>;
  getGains(): Promise<Gains>;
  setGain(loop: Loop, term: Term, value: number): Promise<CommandResult>;
}

const SERVICE_TIMEOUT_MS = 3000;

interface OdometryMessage {
  pose: {
    pose: {
      position: { x: number; y: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
  };
}

type Listener<T> = (value: T) => void;

/** RobotClient over rosbridge (roslib), reconnecting every few seconds after a drop. */
export class RosbridgeClient implements RobotClient {
  private readonly ros = new Ros({});
  private readonly cmdVel: Topic<Twist>;
  private readonly balanceState: Topic<BalanceState>;
  private readonly odometry: Topic<OdometryMessage>;
  private readonly connectionListeners = new Set<Listener<ConnectionStatus>>();
  private readonly stateListeners = new Set<Listener<BalanceState>>();
  private readonly poseListeners = new Set<Listener<Pose2D>>();
  private reconnectTimer: ReturnType<typeof setTimeout> | undefined;
  private wanted = false;

  constructor(private readonly url: string) {
    this.cmdVel = new Topic({
      ros: this.ros,
      name: "/cmd_vel",
      messageType: "geometry_msgs/msg/Twist",
    });
    this.balanceState = new Topic({
      ros: this.ros,
      name: "/balance/state",
      messageType: "robot_interfaces/msg/BalanceState",
      throttle_rate: 40,
    });
    this.odometry = new Topic({
      ros: this.ros,
      name: "/odom",
      messageType: "nav_msgs/msg/Odometry",
      throttle_rate: 100,
    });

    this.ros.on("connection", () => {
      this.emitConnection("connected");
      this.balanceState.subscribe((state) => this.stateListeners.forEach((l) => l(state)));
      this.odometry.subscribe((odom) => {
        const { position, orientation } = odom.pose.pose;
        const pose = { x: position.x, y: position.y, heading: yawFromQuaternion(orientation) };
        this.poseListeners.forEach((l) => l(pose));
      });
    });
    this.ros.on("close", () => {
      this.emitConnection("disconnected");
      this.scheduleReconnect();
    });
    this.ros.on("error", () => {
      this.emitConnection("disconnected");
    });
  }

  connect(): void {
    this.wanted = true;
    this.emitConnection("connecting");
    this.ros.connect(this.url).catch(() => this.scheduleReconnect());
  }

  disconnect(): void {
    this.wanted = false;
    clearTimeout(this.reconnectTimer);
    this.ros.close();
  }

  onConnection(listener: Listener<ConnectionStatus>): () => void {
    return subscribe(this.connectionListeners, listener);
  }

  onBalanceState(listener: Listener<BalanceState>): () => void {
    return subscribe(this.stateListeners, listener);
  }

  onPose(listener: Listener<Pose2D>): () => void {
    return subscribe(this.poseListeners, listener);
  }

  drive(velocity: Velocity): void {
    if (this.ros.isConnected) {
      this.cmdVel.publish(twist(velocity));
    }
  }

  arm(armed: boolean): Promise<CommandResult> {
    return this.call<{ data: boolean }, CommandResult>(
      "/robot_bridge/arm",
      "std_srvs/srv/SetBool",
      { data: armed },
    );
  }

  async getGains(): Promise<Gains> {
    const response = await this.call<
      { names: string[] },
      { values: { type: number; double_value?: number }[] }
    >("/robot_bridge/get_parameters", "rcl_interfaces/srv/GetParameters", {
      names: GAIN_PARAMETER_NAMES,
    });
    return gainsFromResponse(response.values);
  }

  async setGain(loop: Loop, term: Term, value: number): Promise<CommandResult> {
    const response = await this.call<
      unknown,
      { results: { successful: boolean; reason: string }[] }
    >(
      "/robot_bridge/set_parameters",
      "rcl_interfaces/srv/SetParameters",
      setGainRequest(loop, term, value),
    );
    const result = response.results[0];
    return { success: result?.successful ?? false, message: result?.reason ?? "no result" };
  }

  private call<Request, Response>(
    name: string,
    serviceType: string,
    request: Request,
  ): Promise<Response> {
    return new Promise((resolve, reject) => {
      const service = new Service<Request, Response>({ ros: this.ros, name, serviceType });
      service.callService(
        request,
        resolve,
        (error) => reject(new Error(error)),
        SERVICE_TIMEOUT_MS,
      );
    });
  }

  private emitConnection(status: ConnectionStatus): void {
    this.connectionListeners.forEach((l) => l(status));
  }

  private scheduleReconnect(): void {
    if (!this.wanted) {
      return;
    }
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = setTimeout(() => this.connect(), 3000);
  }
}

function subscribe<T>(set: Set<Listener<T>>, listener: Listener<T>): () => void {
  set.add(listener);
  return () => set.delete(listener);
}
