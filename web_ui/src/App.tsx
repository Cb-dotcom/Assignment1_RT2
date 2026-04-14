import { useEffect, useMemo, useRef, useState } from "react";
import { RosDashboardClient } from "./lib/ros";
import type { MotionUiStatus } from "./types/ros";
import { formatPose, yawToQuaternion } from "./types/ros";

type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

const DEFAULT_URL = import.meta.env.VITE_ROSBRIDGE_URL ?? "ws://localhost:9090";

function App() {
  const clientRef = useRef<RosDashboardClient | null>(null);

  const [urlInput, setUrlInput] = useState(DEFAULT_URL);
  const [activeUrl, setActiveUrl] = useState(DEFAULT_URL);

  const [connectionState, setConnectionState] =
    useState<ConnectionState>("disconnected");
  const [connectionMessage, setConnectionMessage] = useState("Not connected.");

  const [status, setStatus] = useState<MotionUiStatus | null>(null);
  const [events, setEvents] = useState<string[]>(["UI initialized."]);

  const [xInput, setXInput] = useState("2.0");
  const [yInput, setYInput] = useState("2.0");
  const [yawInput, setYawInput] = useState("0.0");

  const appendEvent = (message: string) => {
    const time = new Date().toLocaleTimeString();
    setEvents((previous) => [`${time}  ${message}`, ...previous].slice(0, 30));
  };

  useEffect(() => {
    const client = new RosDashboardClient(activeUrl, {
      onConnectionStateChange: (state, message) => {
        setConnectionState(state);
        setConnectionMessage(message);
        appendEvent(`ROS connection state: ${state} (${message})`);
      },
      onStatus: (nextStatus) => {
        setStatus(nextStatus);
        appendEvent(`Status update: ${nextStatus.state} - ${nextStatus.message}`);
      },
    });

    client.subscribeToStatus();
    client.connect();
    clientRef.current = client;

    return () => {
      client.dispose();
      clientRef.current = null;
    };
  }, [activeUrl]);

  const handleReconnect = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();
    setActiveUrl(urlInput.trim());
  };

  const handleSendGoal = (event: React.FormEvent<HTMLFormElement>) => {
    event.preventDefault();

    const x = Number.parseFloat(xInput);
    const y = Number.parseFloat(yInput);
    const yaw = Number.parseFloat(yawInput);

    if (![x, y, yaw].every(Number.isFinite)) {
      appendEvent("Goal rejected locally: x, y, and yaw must be valid numbers.");
      return;
    }

    clientRef.current?.publishGoal({
      header: {
        stamp: { sec: 0, nanosec: 0 },
        frame_id: "map",
      },
      pose: {
        position: { x, y, z: 0.0 },
        orientation: yawToQuaternion(yaw),
      },
    });

    appendEvent(
      `Published goal request in map: x=${x.toFixed(3)}, y=${y.toFixed(
        3,
      )}, yaw=${yaw.toFixed(3)} rad.`,
    );
  };

  const statusColorClass = useMemo(() => {
    switch (status?.state) {
      case "executing":
      case "accepted":
      case "goal_submitted":
        return "badge badge-warning";
      case "succeeded":
        return "badge badge-success";
      case "failed":
      case "rejected":
      case "transform_failed":
      case "server_unavailable":
      case "request_invalid":
        return "badge badge-error";
      case "busy":
        return "badge badge-warning";
      default:
        return "badge badge-neutral";
    }
  }, [status?.state]);

  return (
    <div className="app-shell">
      <header className="topbar">
        <div>
          <h1>RT2 Motion Dashboard</h1>
          <p className="subtitle">
            Web UI skeleton for target-pose control through rosbridge
          </p>
        </div>

        <div className="connection-panel">
          <span className={statusColorClass}>
            {status?.state ?? connectionState}
          </span>
          <span className="connection-text">{connectionMessage}</span>
        </div>
      </header>

      <main className="dashboard-grid">
        <section className="card gazebo-card">
          <div className="card-header">
            <h2>Simulation View</h2>
            <span className="card-tag">Next step</span>
          </div>
          <div className="gazebo-placeholder">
            <p>Gazebo web view will be embedded here in the next step.</p>
            <p>
              For now, this panel reserves the layout and keeps the operator
              console shape stable.
            </p>
          </div>
        </section>

        <section className="side-column">
          <section className="card">
            <div className="card-header">
              <h2>ROS Bridge</h2>
            </div>

            <form onSubmit={handleReconnect} className="stack">
              <label className="field">
                <span>WebSocket URL</span>
                <input
                  value={urlInput}
                  onChange={(event) => setUrlInput(event.target.value)}
                  placeholder="ws://localhost:9090"
                />
              </label>

              <button type="submit" className="primary-button">
                Reconnect
              </button>
            </form>
          </section>

          <section className="card">
            <div className="card-header">
              <h2>Send Goal</h2>
              <span className="card-tag">map frame</span>
            </div>

            <form onSubmit={handleSendGoal} className="stack">
              <label className="field">
                <span>X</span>
                <input
                  value={xInput}
                  onChange={(event) => setXInput(event.target.value)}
                />
              </label>

              <label className="field">
                <span>Y</span>
                <input
                  value={yInput}
                  onChange={(event) => setYInput(event.target.value)}
                />
              </label>

              <label className="field">
                <span>Yaw (rad)</span>
                <input
                  value={yawInput}
                  onChange={(event) => setYawInput(event.target.value)}
                />
              </label>

              <button type="submit" className="primary-button">
                Send Goal
              </button>
            </form>
          </section>

          <section className="card">
            <div className="card-header">
              <h2>Robot Status</h2>
            </div>

            <div className="status-grid">
              <div className="status-row">
                <span>State</span>
                <strong>{status?.state ?? "N/A"}</strong>
              </div>

              <div className="status-row">
                <span>Message</span>
                <strong>{status?.message ?? "No status received yet."}</strong>
              </div>

              <div className="status-row">
                <span>Goal active</span>
                <strong>{status?.goal_active ? "Yes" : "No"}</strong>
              </div>

              <div className="status-row">
                <span>Remaining distance</span>
                <strong>
                  {status ? `${status.remaining_distance.toFixed(3)} m` : "N/A"}
                </strong>
              </div>

              <div className="status-row status-row-block">
                <span>Current pose</span>
                <strong>{status ? formatPose(status.current_pose) : "N/A"}</strong>
              </div>

              <div className="status-row status-row-block">
                <span>Requested goal</span>
                <strong>
                  {status ? formatPose(status.requested_goal_pose) : "N/A"}
                </strong>
              </div>

              <div className="status-row status-row-block">
                <span>Execution goal</span>
                <strong>
                  {status ? formatPose(status.execution_goal_pose) : "N/A"}
                </strong>
              </div>
            </div>
          </section>

          <section className="card">
            <div className="card-header">
              <h2>Event Console</h2>
            </div>

            <div className="event-console">
              {events.map((entry, index) => (
                <div key={`${entry}-${index}`} className="event-line">
                  {entry}
                </div>
              ))}
            </div>
          </section>
        </section>
      </main>
    </div>
  );
}

export default App;