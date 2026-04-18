import { StatusBadge, statusToneFromState } from "./StatusBadge";

type SimulationPaneProps = {
  gazeboUrl: string;
  stateLabel: string;
  operatorStateText: string;
  statusStampText: string;
};

export function SimulationPane({
  gazeboUrl,
  stateLabel,
  operatorStateText,
  statusStampText,
}: SimulationPaneProps) {
  const hasGazeboUrl = gazeboUrl.trim().length > 0;

  return (
    <div className="simulation-shell">
      {hasGazeboUrl ? (
        <iframe
          className="simulation-frame"
          src={gazeboUrl}
          title="Gazebo Web Visualization"
        />
      ) : (
        <div className="simulation-empty-state">
          <p>Gazebo web visualization is not configured yet.</p>
          <p>
            Set a Gazebo Web URL in the Connections panel or define
            VITE_GAZEBO_WEB_URL in your frontend environment.
          </p>
        </div>
      )}

      <div className="simulation-overlay">
        <StatusBadge
          label={stateLabel}
          tone={statusToneFromState(stateLabel)}
        />
        <p>{operatorStateText}</p>
        <div className="simulation-meta">
          <span>Execution frame: odom</span>
          <span>User frame: map</span>
          <span>Status stamp: {statusStampText}</span>
        </div>
      </div>
    </div>
  );
}