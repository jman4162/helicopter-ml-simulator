/**
 * Compact live strip-chart for telemetry (e.g. attitude over time) — the
 * "time-series" educational view. Fixed vertical range, scrolling history.
 */
export interface Series {
  color: string;
  label: string;
}

export class StripChart {
  private readonly ctx: CanvasRenderingContext2D;
  private readonly data: number[][]; // [series][sample]
  private readonly width: number;
  private readonly height: number;

  constructor(
    canvas: HTMLCanvasElement,
    private readonly series: Series[],
    private readonly range: number, // symmetric ± range for the y-axis
    private readonly capacity = 240,
  ) {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.width = canvas.clientWidth || 240;
    this.height = canvas.clientHeight || 90;
    canvas.width = this.width * dpr;
    canvas.height = this.height * dpr;
    const ctx = canvas.getContext('2d')!;
    ctx.scale(dpr, dpr);
    this.ctx = ctx;
    this.data = series.map(() => []);
  }

  push(values: number[]): void {
    for (let i = 0; i < this.series.length; i++) {
      this.data[i].push(values[i] ?? 0);
      if (this.data[i].length > this.capacity) this.data[i].shift();
    }
    this.draw();
  }

  clear(): void {
    for (const d of this.data) d.length = 0;
    this.ctx.clearRect(0, 0, this.width, this.height);
  }

  private draw(): void {
    const { ctx, width: w, height: h, range } = this;
    ctx.clearRect(0, 0, w, h);
    const mid = h / 2;

    // Zero line.
    ctx.strokeStyle = 'rgba(255,255,255,0.18)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, mid);
    ctx.lineTo(w, mid);
    ctx.stroke();

    const n = this.data[0]?.length ?? 0;
    if (n < 2) return;
    const dx = w / (this.capacity - 1);
    const y = (v: number) => mid - (Math.max(-range, Math.min(range, v)) / range) * (mid - 2);

    for (let s = 0; s < this.series.length; s++) {
      const d = this.data[s];
      ctx.strokeStyle = this.series[s].color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      const offset = w - (d.length - 1) * dx; // right-align newest sample
      for (let i = 0; i < d.length; i++) {
        const px = offset + i * dx;
        i === 0 ? ctx.moveTo(px, y(d[i])) : ctx.lineTo(px, y(d[i]));
      }
      ctx.stroke();
    }
  }
}
