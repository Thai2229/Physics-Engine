'use client';

import { useEffect, useRef } from 'react';
import type { MutableRefObject } from 'react';
import type { Telemetry } from '../render/telemetry';

const SERIES = [
  { key: 'kinetic', color: '#16a34a', label: 'Kinetic' },
  { key: 'potential', color: '#2563eb', label: 'Potential' },
  { key: 'total', color: '#dc2626', label: 'Total' },
] as const;

export function EnergyChart({ telemetry }: { telemetry: MutableRefObject<Telemetry> }) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    const ctx = canvas?.getContext('2d');
    if (!canvas || !ctx) {
      return;
    }
    const draw = () => {
      const dpr = window.devicePixelRatio || 1;
      const w = canvas.clientWidth;
      const h = canvas.clientHeight;
      if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
        canvas.width = Math.floor(w * dpr);
        canvas.height = Math.floor(h * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.fillStyle = '#fff';
      ctx.fillRect(0, 0, w, h);
      const samples = telemetry.current.samples;
      if (samples.length < 2) {
        return;
      }
      let min = Infinity;
      let max = -Infinity;
      for (const s of samples) {
        min = Math.min(min, s.kinetic, s.potential, s.total);
        max = Math.max(max, s.kinetic, s.potential, s.total);
      }
      if (max - min < 1e-6) {
        max = min + 1;
      }
      const pad = (max - min) * 0.08;
      min -= pad;
      max += pad;
      const t0 = samples[0].t;
      const t1 = samples[samples.length - 1].t;
      const span = Math.max(t1 - t0, 1e-6);
      ctx.strokeStyle = '#e2e8f0';
      if (min < 0 && max > 0) {
        const y0 = h - ((0 - min) / (max - min)) * h;
        ctx.beginPath();
        ctx.moveTo(0, y0);
        ctx.lineTo(w, y0);
        ctx.stroke();
      }
      for (const series of SERIES) {
        ctx.strokeStyle = series.color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        samples.forEach((s, i) => {
          const x = ((s.t - t0) / span) * w;
          const y = h - ((s[series.key] - min) / (max - min)) * h;
          if (i === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        ctx.stroke();
      }
      ctx.fillStyle = '#475569';
      ctx.font = '10px sans-serif';
      ctx.fillText(`${max.toFixed(1)} J`, 4, 11);
      ctx.fillText(`${min.toFixed(1)} J`, 4, h - 4);
    };
    const timer = setInterval(draw, 100);
    draw();
    return () => clearInterval(timer);
  }, [telemetry]);

  return (
    <div className="chart">
      <div className="legend">
        {SERIES.map((s) => (
          <span key={s.key}>
            <i style={{ background: s.color }} />
            {s.label}
          </span>
        ))}
      </div>
      <canvas ref={ref} className="chart-canvas" />
    </div>
  );
}
