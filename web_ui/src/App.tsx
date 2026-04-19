import { useEffect, useMemo, useRef, useState } from "react";
import { PanelCard } from "./components/PanelCard";
import { SimulationPane } from "./components/SimulationPane";
import { StatusBadge, statusToneFromState } from "./components/StatusBadge";
import { RosDashboardClient } from "./lib/ros";
import type { MotionUiStatus, PoseStamped } from "./types/ros";
import { formatPose, quaternionToYaw, yawToQuaternion } from "./types/ros";

type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

type GoalHistoryEntry = {
  id: string;
  source: "manual" | "preset";
  submittedAtLabel: string;
  x: number;
  y: number;
  yaw: number;
  state: string;
};

type GoalDraft = {
  x: string;
  y: string;
  yaw: string;
};

const DEFAULT_ROSBRIDGE_URL =
  import.meta.env.VITE_ROSBRIDGE_URL ?? "ws://127.0.0.1:9090";

const DEFAULT_GAZEBO_WEB_URL =
  import.meta.env.VITE_GAZEBO_WEB_URL ?? "http://127.0.0.1:3001/visualization";

const QUICK_GOALS: Array<{
  label: string;
  description: string;
  x: number;
  y: number;
  yaw: number;
}> = [
  { label: "Dock", description: "Return to the nominal origin", x: 0.0, y: 0.0, yaw: 0.0 },
  { label: "North-East", description: "Move to the upper right test position", x: 2.0, y: 2.0, yaw: 0.0 },
  { label: "Inspection", description: "Face the upper corridor", x: 1.5, y: 3.0, yaw: 1.57 },
];

function formatClockTime(date: Date): string {
  return date.toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

function formatStatusStamp(status: MotionUiStatus | null): string {
  if (!status) {
    return "No status received";
  }

  return `${status.stamp.sec}.${status.stamp.nanosec.toString().padStart(9, "0")}`;
}

function parseGoalDraft(draft: GoalDraft): { x: number; y: number; yaw: number } | null {
  const x = Number.parseFloat(draft.x);
  const y = Number.parseFloat(draft.y);
  const yaw = Number.parseFloat(draft.yaw);

  if (![x, y, yaw].every(Number.isFinite)) {
    return null;
  }

  return { x, y, yaw };
}

function buildPoseStamped(x: number, y: number, yaw: number): PoseStamped {
  return {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: "map",
    },
    pose: {
      position: { x, y, z: 0.0 },
      orientation: yawToQuaternion(yaw),
    },
  };
}

function App() {
  const clientRef = useRef<RosDashboardClient | null>(null);
  const lastStatusEventKeyRef = useRef<string>("");

  const [rosbridgeUrlInput, setRosbridgeUrlInput] = useState(DEFAULT_ROSBRIDGE_URL);
  const [activeRosbridgeUrl, setActiveRosbridgeUrl] = useState(DEFAULT_ROSBRIDGE_URL);

  const [rosReconnectCounter, setRosReconnectCounter] = useState(0);

  const [connectionState, setConnectionState] = useState<ConnectionState>("disconnected");
  const [connectionMessage, setConnectionMessage] = useState("Not connected.");

  const [status, setStatus] = useState<MotionUiStatus | null>(null);
  const [events, setEvents] = useState<string[]>(["UI initialized."]);

  const [goalDraft, setGoalDraft] = useState<GoalDraft>({
    x: "2.0",
    y: "2.0",
    yaw: "0.0",
  });

  const [goalHistory, setGoalHistory] = useState<GoalHistoryEntry[]>([]);

  const appendEvent = (message: string) => {
    const timestamp = formatClockTime(new Date());
    setEvents((previous) => [`${timestamp}  ${message}`, ...previous].slice(0, 40));
  };

  useEffect(() => {
    const client = new RosDashboardClient(activeRosbridgeUrl, {
      onConnectionStateChange: (state, message) => {
        setConnectionState(state);
        setConnectionMessage(message);
        appendEvent(`ROS connection state changed to "${state}". ${message}`);
      },
      onStatus: (nextStatus) => {
        setStatus(nextStatus);

        const nextEventKey = `${nextStatus.state}::${nextStatus.message}`;
        if (lastStatusEventKeyRef.current !== nextEventKey) {
          lastStatusEventKeyRef.current = nextEventKey;
          appendEvent(`Motion status: ${nextStatus.state} — ${nextStatus.message}`);
        }

        setGoalHistory((previous) => {
          if (previous.length === 0) {
            return previous;
          }

          const [latest, ...rest] = previous;
          return [{ ...latest, state: nextStatus.state }, ...rest];
        });
      },
    });

    client.subscribeToStatus();
    client.connect();
    clientRef.current = client;

    return () => {
      client.dispose();
      clientRef.current = null;
    };
  }, [activeRosbridgeUrl, rosReconnectCounter]);

  const handleApplyConnections = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();

    const nextRosbridgeUrl = rosbridgeUrlInput.trim();

    if (nextRosbridgeUrl.length === 0) {
      appendEvent("Connection update ignored: rosbridge URL cannot be empty.");
      return;
    }

    setActiveRosbridgeUrl(nextRosbridgeUrl);
    setRosReconnectCounter((previous) => previous + 1);

    appendEvent(
      `Applied connection settings. rosbridge=${nextRosbridgeUrl}, gazebo=${DEFAULT_GAZEBO_WEB_URL}`,
    );
  };

  const submitGoal = (
    values: { x: number; y: number; yaw: number },
    source: "manual" | "preset",
    label: string,
  ) => {
    const goal = buildPoseStamped(values.x, values.y, values.yaw);
    clientRef.current?.publishGoal(goal);

    setGoalHistory((previous) => [
      {
        id: `${Date.now()}`,
        source,
        submittedAtLabel: formatClockTime(new Date()),
        x: values.x,
        y: values.y,
        yaw: values.yaw,
        state: "goal_submitted",
      },
      ...previous,
    ].slice(0, 8));

    appendEvent(
      `Published ${label} goal in map frame: x=${values.x.toFixed(3)}, y=${values.y.toFixed(
        3,
      )}, yaw=${values.yaw.toFixed(3)} rad.`,
    );
  };

  const handleManualSubmit = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();

    const parsed = parseGoalDraft(goalDraft);
    if (!parsed) {
      appendEvent("Manual goal rejected locally: x, y, and yaw must be valid numbers.");
      return;
    }

    submitGoal(parsed, "manual", "manual");
  };

  const handlePresetClick = (preset: (typeof QUICK_GOALS)[number]) => {
    setGoalDraft({
      x: preset.x.toString(),
      y: preset.y.toString(),
      yaw: preset.yaw.toString(),
    });

    submitGoal(
      { x: preset.x, y: preset.y, yaw: preset.yaw },
      "preset",
      `preset "${preset.label}"`,
    );
  };

  const canSendGoal = useMemo(() => {
    return connectionState === "connected" && !(status?.goal_active ?? false);
  }, [connectionState, status?.goal_active]);

  const operatorStateText = useMemo(() => {
    if (status?.goal_active) {
      return "Robot is executing a goal.";
    }

    if (status?.state === "succeeded") {
      return "Robot reached the goal and is ready for the next command.";
    }

    if (connectionState !== "connected") {
      return "Dashboard is waiting for a live rosbridge connection.";
    }

    return "System is ready to accept a new target pose.";
  }, [connectionState, status?.goal_active, status?.state]);

  const currentYaw = useMemo(() => {
    if (!status) {
      return null;
    }
    return quaternionToYaw(status.current_pose.pose.orientation);
  }, [status]);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div className="topbar-text">
          <h1>RT2 Motion Operator Console</h1>
          <p className="subtitle">
            Target-pose command dashboard for the composed ROS 2 motion stack
          </p>
        </div>

        <div className="topbar-status">
          <StatusBadge
            label={status?.state ?? connectionState}
            tone={statusToneFromState(status?.state ?? connectionState)}
          />
          <span className="connection-text">{connectionMessage}</span>
        </div>
      </header>

      <main className="dashboard-grid">
        <section className="left-column">
          <PanelCard
            title="Simulation View"
            subtitle="Embedded browser surface for Gazebo web visualization"
            accent={<span className="card-tag">embedded</span>}
            className="simulation-card"
          >
            <SimulationPane
              gazeboUrl={DEFAULT_GAZEBO_WEB_URL}
              stateLabel={status?.state ?? "idle"}
              operatorStateText={operatorStateText}
              statusStampText={formatStatusStamp(status)}
            />
          </PanelCard>

          <PanelCard
            title="Execution Snapshot"
            subtitle="Primary live motion indicators for the operator"
          >
            <div className="metrics-grid">
              <div className="metric-tile">
                <span className="metric-label">Goal active</span>
                <strong className="metric-value">{status?.goal_active ? "Yes" : "No"}</strong>
              </div>

              <div className="metric-tile">
                <span className="metric-label">Remaining distance</span>
                <strong className="metric-value">
                  {status ? `${status.remaining_distance.toFixed(3)} m` : "N/A"}
                </strong>
              </div>

              <div className="metric-tile">
                <span className="metric-label">Current yaw</span>
                <strong className="metric-value">
                  {currentYaw !== null ? `${currentYaw.toFixed(3)} rad` : "N/A"}
                </strong>
              </div>

              <div className="metric-tile">
                <span className="metric-label">Last update</span>
                <strong className="metric-value">{formatStatusStamp(status)}</strong>
              </div>
            </div>

            <div className="pose-summary-grid">
              <div className="pose-summary-block">
                <span>Current pose</span>
                <strong>{status ? formatPose(status.current_pose) : "No pose received yet."}</strong>
              </div>

              <div className="pose-summary-block">
                <span>Requested goal</span>
                <strong>
                  {status ? formatPose(status.requested_goal_pose) : "No goal requested yet."}
                </strong>
              </div>

              <div className="pose-summary-block">
                <span>Execution goal</span>
                <strong>
                  {status ? formatPose(status.execution_goal_pose) : "No execution goal yet."}
                </strong>
              </div>
            </div>
          </PanelCard>
        </section>

        <section className="right-column">
          <PanelCard
            title="Connections"
            subtitle="ROS endpoint plus fixed local Gazebo viewer"
          >
            <form onSubmit={handleApplyConnections} className="stack">
              <label className="field">
                <span>ROS Bridge WebSocket URL</span>
                <input
                  value={rosbridgeUrlInput}
                  onChange={(event) => setRosbridgeUrlInput(event.target.value)}
                  placeholder="ws://127.0.0.1:9090"
                />
              </label>

              <label className="field">
                <span>Gazebo Web URL</span>
                <input
                  value={DEFAULT_GAZEBO_WEB_URL}
                  readOnly
                />
              </label>

              <button type="submit" className="secondary-button">
                Apply Connections
              </button>
            </form>
          </PanelCard>

          <PanelCard
            title="Send Goal"
            subtitle="Publish a target pose in the map frame"
            accent={
              <StatusBadge
                label={canSendGoal ? "ready" : "locked"}
                tone={canSendGoal ? "success" : "warning"}
              />
            }
          >
            <div className="preset-grid">
              {QUICK_GOALS.map((preset) => (
                <button
                  key={preset.label}
                  type="button"
                  className="preset-button"
                  disabled={!canSendGoal}
                  onClick={() => handlePresetClick(preset)}
                >
                  <strong>{preset.label}</strong>
                  <span>{preset.description}</span>
                </button>
              ))}
            </div>

            <form onSubmit={handleManualSubmit} className="stack manual-goal-form">
              <label className="field">
                <span>X</span>
                <input
                  value={goalDraft.x}
                  onChange={(event) =>
                    setGoalDraft((previous) => ({ ...previous, x: event.target.value }))
                  }
                />
              </label>

              <label className="field">
                <span>Y</span>
                <input
                  value={goalDraft.y}
                  onChange={(event) =>
                    setGoalDraft((previous) => ({ ...previous, y: event.target.value }))
                  }
                />
              </label>

              <label className="field">
                <span>Yaw (rad)</span>
                <input
                  value={goalDraft.yaw}
                  onChange={(event) =>
                    setGoalDraft((previous) => ({ ...previous, yaw: event.target.value }))
                  }
                />
              </label>

              <button type="submit" className="primary-button" disabled={!canSendGoal}>
                {status?.goal_active ? "Goal in progress" : "Send Goal"}
              </button>
            </form>
          </PanelCard>

          <PanelCard
            title="Goal History"
            subtitle="Most recent operator-issued commands"
          >
            <div className="history-list">
              {goalHistory.length === 0 ? (
                <div className="empty-state">No goals have been sent from the Web UI yet.</div>
              ) : (
                goalHistory.map((entry) => (
                  <div key={entry.id} className="history-row">
                    <div>
                      <strong>
                        {entry.source === "preset" ? "Preset goal" : "Manual goal"}
                      </strong>
                      <span>
                        {entry.submittedAtLabel} · x={entry.x.toFixed(2)}, y={entry.y.toFixed(2)}, yaw=
                        {entry.yaw.toFixed(2)}
                      </span>
                    </div>

                    <StatusBadge
                      label={entry.state}
                      tone={statusToneFromState(entry.state)}
                    />
                  </div>
                ))
              )}
            </div>
          </PanelCard>

          <PanelCard
            title="Event Console"
            subtitle="Curated high-level UI and motion events"
          >
            <div className="event-console">
              {events.map((entry, index) => (
                <div key={`${entry}-${index}`} className="event-line">
                  {entry}
                </div>
              ))}
            </div>
          </PanelCard>
        </section>
      </main>
    </div>
  );
}

export default App;