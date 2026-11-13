'use client';

import { LIMITS, MAX_LINKS, MIN_LINKS } from '../scenes/sceneState';
import type { LinkParams } from '../scenes/sceneState';

interface NumberFieldProps {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  unit?: string;
  onChange: (v: number) => void;
}

export function NumberField({ label, value, min, max, step, unit, onChange }: NumberFieldProps) {
  return (
    <label className="field">
      <span>
        {label}: {Number(value.toFixed(2))}
        {unit}
      </span>
      <input type="range" min={min} max={max} step={step} value={value} onChange={(e) => onChange(Number(e.target.value))} />
    </label>
  );
}

interface ArmEditorProps {
  links: LinkParams[];
  onChange: (links: LinkParams[]) => void;
  onAdd: () => void;
  onRemove: () => void;
}

export function ArmEditor({ links, onChange, onAdd, onRemove }: ArmEditorProps) {
  const update = (i: number, patch: Partial<LinkParams>) => {
    const next = links.map((l, k) => {
      if (k === i) {
        return { ...l, ...patch };
      }
      return l;
    });
    onChange(next);
  };
  return (
    <section>
      <h3>Arm ({links.length} links)</h3>
      <div className="row">
        <button onClick={onAdd} disabled={links.length >= MAX_LINKS}>
          + Add link
        </button>
        <button onClick={onRemove} disabled={links.length <= MIN_LINKS}>
          - Remove link
        </button>
      </div>
      {links.map((l, i) => (
        <fieldset key={i}>
          <legend>Link {i + 1}</legend>
          <NumberField label="Length" unit=" m" value={l.length} {...LIMITS.length} step={0.05} onChange={(v) => update(i, { length: v })} />
          <NumberField label="Width" unit=" m" value={l.width} {...LIMITS.width} step={0.01} onChange={(v) => update(i, { width: v })} />
          <NumberField label="Density" unit=" kg/m²" value={l.density} {...LIMITS.density} step={0.1} onChange={(v) => update(i, { density: v })} />
          <label className="check">
            <input type="checkbox" checked={l.limitEnabled} onChange={(e) => update(i, { limitEnabled: e.target.checked })} />
            Joint limit
          </label>
          {l.limitEnabled && (
            <>
              <NumberField
                label="Lower"
                unit="°"
                value={l.lowerDeg}
                {...LIMITS.angleDeg}
                step={1}
                onChange={(v) => update(i, { lowerDeg: Math.min(v, l.upperDeg) })}
              />
              <NumberField
                label="Upper"
                unit="°"
                value={l.upperDeg}
                {...LIMITS.angleDeg}
                step={1}
                onChange={(v) => update(i, { upperDeg: Math.max(v, l.lowerDeg) })}
              />
            </>
          )}
          <label className="check">
            <input type="checkbox" checked={l.motorEnabled} onChange={(e) => update(i, { motorEnabled: e.target.checked })} />
            Joint motor
          </label>
          {l.motorEnabled && (
            <>
              <NumberField label="Speed" unit=" rad/s" value={l.motorSpeed} {...LIMITS.motorSpeed} step={0.1} onChange={(v) => update(i, { motorSpeed: v })} />
              <NumberField label="Max torque" unit=" N·m" value={l.maxTorque} {...LIMITS.maxTorque} step={1} onChange={(v) => update(i, { maxTorque: v })} />
            </>
          )}
        </fieldset>
      ))}
    </section>
  );
}
