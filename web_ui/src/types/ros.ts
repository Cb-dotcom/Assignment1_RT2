export interface Time {
  sec: number;
  nanosec: number;
}

export interface Header {
  stamp: Time;
  frame_id: string;
}

export interface Point {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface Pose {
  position: Point;
  orientation: Quaternion;
}

export interface PoseStamped {
  header: Header;
  pose: Pose;
}

export interface MotionUiStatus {
  stamp: Time;
  state: string;
  message: string;
  goal_active: boolean;
  remaining_distance: number;
  current_pose: PoseStamped;
  requested_goal_pose: PoseStamped;
  execution_goal_pose: PoseStamped;
}

export function yawToQuaternion(yaw: number): Quaternion {
  return {
    x: 0.0,
    y: 0.0,
    z: Math.sin(yaw / 2.0),
    w: Math.cos(yaw / 2.0),
  };
}

export function quaternionToYaw(quaternion: Quaternion): number {
  const sinyCosp =
    2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const cosyCosp =
    1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);

  return Math.atan2(sinyCosp, cosyCosp);
}

export function formatPose(pose: PoseStamped): string {
  const yaw = quaternionToYaw(pose.pose.orientation);

  return `${pose.header.frame_id} | x=${pose.pose.position.x.toFixed(
    3,
  )}, y=${pose.pose.position.y.toFixed(3)}, yaw=${yaw.toFixed(3)} rad`;
}