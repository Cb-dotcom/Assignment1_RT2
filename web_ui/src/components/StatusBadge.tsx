type StatusTone = "success" | "warning" | "error" | "neutral";

export function statusToneFromState(state: string): StatusTone {
  switch (state) {
    case "succeeded":
    case "ready":
    case "connected":
      return "success";

    case "executing":
    case "accepted":
    case "goal_submitted":
    case "busy":
    case "connecting":
    case "locked":
      return "warning";

    case "failed":
    case "rejected":
    case "transform_failed":
    case "server_unavailable":
    case "request_invalid":
    case "error":
    case "disconnected":
    case "canceled":
      return "error";

    default:
      return "neutral";
  }
}

export function StatusBadge({
  label,
  tone,
}: {
  label: string;
  tone: StatusTone;
}) {
  return <span className={`status-badge status-badge-${tone}`}>{label}</span>;
}