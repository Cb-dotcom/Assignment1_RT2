import { StatusBadge, statusToneFromState } from "./StatusBadge";

type SimulationPaneProps = {
  gazeboUrl: string;
  stateLabel: string;
  operatorStateText: string;
  statusStampText: string;
  focused: boolean;
  onToggleFocus: () => void;
};

export function SimulationPane({
  gazeboUrl,
  stateLabel,
  operatorStateText,
  statusStampText,
  focused,
  onToggleFocus,
}: SimulationPaneProps) {
  const hasGazeboUrl = gazeboUrl.trim().length > 0;

  return (
    <div className={`simulation-shell ${focused ? "simulation-shell-focused" : ""}`}>
      {hasGazeboUrl ? (
        <div className="simulation-crop-frame">
          <iframe
            className="simulation-frame simulation-frame-cropped"
            src={gazeboUrl}
            title="Gazebo Web Visualization"
            allow="fullscreen"
            allowFullScreen
          />
        </div>
      ) : (
        <div className="simulation-empty-state">
          <p>Gazebo web visualization is not configured yet.</p>
          <p>Set VITE_GAZEBO_WEB_URL in the frontend environment.</p>
        </div>
      )}

      <div className="simulation-top-controls">
        <button
          type="button"
          className="simulation-focus-button"
          onClick={onToggleFocus}
        >
          {focused ? "Restore view" : "Expand view"}
        </button>
      </div>

      <div className="simulation-status-chip">
        <StatusBadge
          label={stateLabel}
          tone={statusToneFromState(stateLabel)}
        />
        <span className="simulation-status-text">{operatorStateText}</span>
        <small className="simulation-status-stamp">{statusStampText}</small>
      </div>
    </div>
  );
}