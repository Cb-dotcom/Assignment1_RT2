import * as ROSLIB from "roslib";
import type { MotionUiStatus, PoseStamped } from "../types/ros";

type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

interface RosDashboardCallbacks {
  onConnectionStateChange?: (state: ConnectionState, message: string) => void;
  onStatus?: (status: MotionUiStatus) => void;
}

const GOAL_TOPIC_NAME = "/target_pose_requests";
const GOAL_TOPIC_TYPE = "geometry_msgs/msg/PoseStamped";

export class RosDashboardClient {
  private readonly url: string;
  private readonly ros: ROSLIB.Ros;
  private readonly statusTopic: ROSLIB.Topic;
  private readonly callbacks: RosDashboardCallbacks;

  private subscribed = false;
  private connected = false;

  constructor(url: string, callbacks: RosDashboardCallbacks = {}) {
    this.url = url;
    this.callbacks = callbacks;

    this.ros = new ROSLIB.Ros();

    this.statusTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/motion_ui_status",
      messageType: "bme_gazebo_sensors_interfaces/MotionUiStatus",
      reconnect_on_close: true,
    });

    this.ros.on("connection", () => {
      this.connected = true;
      this.advertiseGoalPublisher();

      this.callbacks.onConnectionStateChange?.(
        "connected",
        `Connected to ${this.url}`,
      );
    });

    this.ros.on("close", () => {
      this.connected = false;
      this.callbacks.onConnectionStateChange?.(
        "disconnected",
        `Disconnected from ${this.url}`,
      );
    });

    this.ros.on("error", (error: unknown) => {
      this.connected = false;
      const message =
        error instanceof Error ? error.message : "Unknown rosbridge error";
      this.callbacks.onConnectionStateChange?.("error", message);
    });
  }

  private advertiseGoalPublisher(): void {
    (this.ros as any).callOnConnection({
      op: "advertise",
      topic: GOAL_TOPIC_NAME,
      type: GOAL_TOPIC_TYPE,
      queue_size: 1,
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

  publishGoal(goal: PoseStamped): boolean {
    if (!this.connected) {
      console.warn("ROS goal publish skipped: rosbridge is not connected.");
      return false;
    }

    this.advertiseGoalPublisher();

    console.log("Publishing goal through rosbridge:", goal);

    (this.ros as any).callOnConnection({
      op: "publish",
      topic: GOAL_TOPIC_NAME,
      type: GOAL_TOPIC_TYPE,
      msg: goal,
    });

    return true;
  }

  dispose(): void {
    if (this.subscribed) {
      this.statusTopic.unsubscribe();
      this.subscribed = false;
    }

    if (this.connected) {
      (this.ros as any).callOnConnection({
        op: "unadvertise",
        topic: GOAL_TOPIC_NAME,
      });
    }

    this.ros.close();
  }
}