import { useEffect, useRef } from "react";

import type { BalanceState } from "../lib/robot";
import { scaleY, TimeSeries } from "../lib/timeSeries";

const WINDOW_S = 10;
const PITCH_RANGE_DEG = 15;
const SERIES = [
  { label: "pitch [°/15]", cssVar: "--series-1" },
  { label: "left motor", cssVar: "--series-2" },
  { label: "right motor", cssVar: "--series-3" },
];

interface Props {
  state: BalanceState | null;
}

/** Last 10 s of pitch (scaled to ±15°) and both motor commands (±1) on one shared axis. */
export function TelemetryChart({ state }: Props) {
  const canvas = useRef<HTMLCanvasElement>(null);
  const series = useRef(new TimeSeries(WINDOW_S));

  useEffect(() => {
    if (!state) {
      return;
    }
    const pitchScaled = (state.pitch * 180) / Math.PI / PITCH_RANGE_DEG;
    series.current.push(performance.now() / 1000, [
      pitchScaled,
      state.left_command,
      state.right_command,
    ]);
    draw(canvas.current, series.current);
  }, [state]);

  return (
    <section className="panel wide" aria-label="Telemetry">
      <h2>Telemetry</h2>
      <canvas ref={canvas} className="chart" width={800} height={220} />
      <ul className="legend">
        {SERIES.map((s) => (
          <li key={s.label}>
            <span className="swatch" style={{ background: `var(${s.cssVar})` }} />
            {s.label}
          </li>
        ))}
      </ul>
    </section>
  );
}

function draw(canvas: HTMLCanvasElement | null, series: TimeSeries) {
  const context = canvas?.getContext("2d");
  const latest = series.latest;
  if (!canvas || !context || !latest) {
    return;
  }
  const { width, height } = canvas;
  const styles = getComputedStyle(canvas);
  context.clearRect(0, 0, width, height);

  context.strokeStyle = styles.getPropertyValue("--grid") || "#8884";
  context.lineWidth = 1;
  context.beginPath();
  for (const level of [-1, 0, 1]) {
    const y = scaleY(level, -1, 1, height);
    context.moveTo(0, y);
    context.lineTo(width, y);
  }
  context.stroke();

  SERIES.forEach((info, index) => {
    context.strokeStyle = styles.getPropertyValue(info.cssVar) || "#4a8";
    context.lineWidth = 2;
    context.beginPath();
    series.data.forEach((sample, i) => {
      const x = width - ((latest.t - sample.t) / WINDOW_S) * width;
      const y = scaleY(sample.values[index], -1, 1, height);
      if (i === 0) {
        context.moveTo(x, y);
      } else {
        context.lineTo(x, y);
      }
    });
    context.stroke();
  });
}
