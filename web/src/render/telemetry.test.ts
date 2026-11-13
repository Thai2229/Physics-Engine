import { describe, expect, it } from 'vitest';
import { createTelemetry, recordSample } from './telemetry';

describe('recordSample', () => {
  it('records raw energy by default', () => {
    const t = createTelemetry();
    recordSample(t, { bodies: 1, contacts: 0, stepMs: 0, kineticEnergy: 2, potentialEnergy: 5 });
    expect(t.samples[0]).toMatchObject({ kinetic: 2, potential: 5, total: 7 });
  });

  it('subtracts the rest potential so a body at rest reads zero total energy', () => {
    const t = createTelemetry();
    recordSample(t, { bodies: 1, contacts: 0, stepMs: 0, kineticEnergy: 0, potentialEnergy: 4.41 }, 4.41);
    expect(t.samples[0].potential).toBeCloseTo(0, 12);
    expect(t.samples[0].total).toBeCloseTo(0, 12);
  });
});
