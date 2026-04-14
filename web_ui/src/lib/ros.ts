import * as ROSLIB from "roslib";
import type { MotionUiStatus, PoseStamped } from "../types/ros";

type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

interface RosDashboardCallbacks {
  onConnectionStateChange?: (state: ConnectionState, message: string) => void;
  onStatus?: (status: MotionUiStatus) => void;
}

export class RosDashboardClient {
  private readonly url: string;
  private readonly ros: ROSLIB.Ros;
  private readonly statusTopic: ROSLIB.Topic;
  private readonly goalTopic: ROSLIB.Topic;
  private readonly callbacks: RosDashboardCallbacks;

  private subscribed = false;

  constructor(url: string, callbacks: RosDashboardCallbacks = {}) {
    this.url = url;
    this.callbacks = callbacks;

    this.ros = new ROSLIB.Ros();

    this.statusTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/motion_ui_status",
      messageType: "bme_gazebo_sensors_interfaces/msg/MotionUiStatus",
    });

    this.goalTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/target_pose_requests",
      messageType: "geometry_msgs/msg/PoseStamped",
    });

    this.ros.on("connection", () => {
      this.callbacks.onConnectionStateChange?.(
        "connected",
        `Connected to ${this.url}`,
      );
    });

    this.ros.on("close", () => {
      this.callbacks.onConnectionStateChange?.(
        "disconnected",
        `Disconnected from ${this.url}`,
      );
    });

    this.ros.on("error", (error: unknown) => {
      const message =
        error instanceof Error ? error.message : "Unknown rosbridge error";
      this.callbacks.onConnectionStateChange?.("error", message);
    });
  }

  connect(): void {
    this.callbacks.onConnectionStateChange?.(
      "connecting",
      `Connecting to ${this.url}`,
    );
    this.ros.connect(this.url);
  }

  subscribeToStatus(): void {
    if (this.subscribed) {
      return;
    }

    this.statusTopic.subscribe((message: MotionUiStatus) => {
      this.callbacks.onStatus?.(message);
    });

    this.subscribed = true;
  }

  publishGoal(goal: PoseStamped): void {
    this.goalTopic.publish(new ROSLIB.Message(goal));
  }

  dispose(): void {
    if (this.subscribed) {
      this.statusTopic.unsubscribe();
      this.subscribed = false;
    }

    this.ros.close();
  }
}