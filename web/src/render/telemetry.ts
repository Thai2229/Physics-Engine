import type { WorldStats } from '../engine/types';

export interface EnergySample {
  t: number;
  kinetic: number;
  potential: number;
  total: number;
}

export interface Telemetry {
  simTime: number;
  samples: EnergySample[];
  stats: WorldStats;
}

export const MAX_SAMPLES = 600;
export const SAMPLE_INTERVAL = 1 / 30;

export function createTelemetry(): Telemetry {
  return {
    simTime: 0,
    samples: [],
    stats: { bodies: 0, contacts: 0, stepMs: 0, kineticEnergy: 0, potentialEnergy: 0 },
  };
}

export function resetTelemetry(t: Telemetry): void {
  t.simTime = 0;
  t.samples.length = 0;
}

/** Records a sample if SAMPLE_INTERVAL of sim time has elapsed since the last one. */
export function recordSample(t: Telemetry, stats: WorldStats): void {
  t.stats = stats;
  const last = t.samples[t.samples.length - 1];
  if (last && t.simTime - last.t < SAMPLE_INTERVAL) {
    return;
  }
  t.samples.push({
    t: t.simTime,
    kinetic: stats.kineticEnergy,
    potential: stats.potentialEnergy,
    total: stats.kineticEnergy + stats.potentialEnergy,
  });
  if (t.samples.length > MAX_SAMPLES) {
    t.samples.shift();
  }
}
