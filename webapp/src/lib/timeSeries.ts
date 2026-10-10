/** Bounded history of samples for live plots. */

export interface Sample {
  t: number; // seconds
  values: number[];
}

/** Keeps the samples of the last `window` seconds (oldest first). */
export class TimeSeries {
  private samples: Sample[] = [];

  constructor(readonly window: number) {}

  push(t: number, values: number[]): void {
    if (!Number.isFinite(t) || values.some((v) => !Number.isFinite(v))) {
      return;
    }
    this.samples.push({ t, values });
    const cutoff = t - this.window;
    let first = 0;
    while (first < this.samples.length && this.samples[first].t < cutoff) {
      first++;
    }
    if (first > 0) {
      this.samples.splice(0, first);
    }
  }

  get data(): readonly Sample[] {
    return this.samples;
  }

  get latest(): Sample | undefined {
    return this.samples[this.samples.length - 1];
  }

  clear(): void {
    this.samples = [];
  }
}

/** Maps a value range to pixel rows (top = max). */
export function scaleY(value: number, min: number, max: number, height: number): number {
  const clamped = Math.max(min, Math.min(max, value));
  return height - ((clamped - min) / (max - min)) * height;
}
