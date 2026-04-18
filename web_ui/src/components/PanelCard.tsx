import type { PropsWithChildren, ReactNode } from "react";

type PanelCardProps = PropsWithChildren<{
  title: string;
  subtitle?: string;
  accent?: ReactNode;
  className?: string;
}>;

export function PanelCard({
  title,
  subtitle,
  accent,
  className,
  children,
}: PanelCardProps) {
  return (
    <section className={`panel-card ${className ?? ""}`.trim()}>
      <header className="panel-card-header">
        <div>
          <h2>{title}</h2>
          {subtitle ? <p>{subtitle}</p> : null}
        </div>

        {accent ? <div className="panel-card-accent">{accent}</div> : null}
      </header>

      <div className="panel-card-body">{children}</div>
    </section>
  );
}