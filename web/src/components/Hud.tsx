'use client';

import { useEffect, useState } from 'react';
import type { MutableRefObject } from 'react';
import type { EngineHandle } from '../engine/loadEngine';
import type { Telemetry } from '../render/telemetry';

export function Hud({ telemetry, handle }: { telemetry: MutableRefObject<Telemetry>; handle: EngineHandle | null }) {
  const [stats, setStats] = useState(telemetry.current.stats);
  useEffect(() => {
    const timer = setInterval(() => setStats({ ...telemetry.current.stats }), 250);
    return () => clearInterval(timer);
  }, [telemetry]);
  let backend = 'loading...';
  if (handle) {
    backend = handle.backend;
  }
  return (
    <div className="hud">
      <div>bodies: {stats.bodies}</div>
      <div>contacts: {stats.contacts}</div>
      <div>step: {stats.stepMs.toFixed(2)} ms</div>
      <div>
        backend: <b className={`backend-${backend}`}>{backend}</b>
      </div>
    </div>
  );
}
