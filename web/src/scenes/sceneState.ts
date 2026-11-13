export const TAB_IDS = ['arm', 'double-pendulum', 'bridge', 'cradle', 'stack'] as const;
export type TabId = (typeof TAB_IDS)[number];

export interface LinkParams {
  length: number;
  width: number;
  density: number;
  limitEnabled: boolean;
  lowerDeg: number;
  upperDeg: number;
  motorEnabled: boolean;
  motorSpeed: number;
  maxTorque: number;
}

export interface SceneState {
  tab: TabId;
  gravity: number;
  timeScale: number;
  links: LinkParams[];
}

export const MIN_LINKS = 1;
export const MAX_LINKS = 8;

export const LIMITS = {
  length: { min: 0.2, max: 3 },
  width: { min: 0.05, max: 0.6 },
  density: { min: 0.1, max: 10 },
  angleDeg: { min: -180, max: 180 },
  motorSpeed: { min: -10, max: 10 },
  maxTorque: { min: 0, max: 500 },
  gravity: { min: 0, max: 30 },
  timeScale: { min: 0.1, max: 2 },
} as const;

export function defaultLink(): LinkParams {
  return {
    length: 1,
    width: 0.2,
    density: 1,
    limitEnabled: false,
    lowerDeg: -90,
    upperDeg: 90,
    motorEnabled: false,
    motorSpeed: 1,
    maxTorque: 20,
  };
}

export function defaultState(): SceneState {
  return {
    tab: 'arm',
    gravity: 9.81,
    timeScale: 1,
    links: [
      { ...defaultLink(), length: 1.4 },
      { ...defaultLink(), length: 1.1 },
      { ...defaultLink(), length: 0.8 },
    ],
  };
}

export function clamp(v: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, v));
}

function num(v: unknown, min: number, max: number, fallback: number): number {
  if (typeof v !== 'number' || !Number.isFinite(v)) {
    return fallback;
  }
  return clamp(v, min, max);
}

function bool(v: unknown, fallback: boolean): boolean {
  if (typeof v !== 'boolean') {
    return fallback;
  }
  return v;
}

function isRecord(v: unknown): v is Record<string, unknown> {
  return typeof v === 'object' && v !== null && !Array.isArray(v);
}

export function sanitizeLink(raw: unknown): LinkParams {
  const d = defaultLink();
  if (!isRecord(raw)) {
    return d;
  }
  const a = num(raw.lowerDeg, LIMITS.angleDeg.min, LIMITS.angleDeg.max, d.lowerDeg);
  const b = num(raw.upperDeg, LIMITS.angleDeg.min, LIMITS.angleDeg.max, d.upperDeg);
  return {
    length: num(raw.length, LIMITS.length.min, LIMITS.length.max, d.length),
    width: num(raw.width, LIMITS.width.min, LIMITS.width.max, d.width),
    density: num(raw.density, LIMITS.density.min, LIMITS.density.max, d.density),
    limitEnabled: bool(raw.limitEnabled, d.limitEnabled),
    lowerDeg: Math.min(a, b),
    upperDeg: Math.max(a, b),
    motorEnabled: bool(raw.motorEnabled, d.motorEnabled),
    motorSpeed: num(raw.motorSpeed, LIMITS.motorSpeed.min, LIMITS.motorSpeed.max, d.motorSpeed),
    maxTorque: num(raw.maxTorque, LIMITS.maxTorque.min, LIMITS.maxTorque.max, d.maxTorque),
  };
}

/** Validates arbitrary parsed JSON into a SceneState. Never throws. */
export function sanitizeState(raw: unknown): SceneState {
  const d = defaultState();
  if (!isRecord(raw)) {
    return d;
  }
  let tab = d.tab;
  if (typeof raw.tab === 'string' && (TAB_IDS as readonly string[]).includes(raw.tab)) {
    tab = raw.tab as TabId;
  }
  let links = d.links;
  if (Array.isArray(raw.links) && raw.links.length >= MIN_LINKS) {
    links = raw.links.slice(0, MAX_LINKS).map(sanitizeLink);
  }
  return {
    tab,
    gravity: num(raw.gravity, LIMITS.gravity.min, LIMITS.gravity.max, d.gravity),
    timeScale: num(raw.timeScale, LIMITS.timeScale.min, LIMITS.timeScale.max, d.timeScale),
    links,
  };
}

const HASH_PREFIX = 's=';

export function serializeState(state: SceneState): string {
  return HASH_PREFIX + encodeURIComponent(JSON.stringify(state));
}

/** Accepts the string with or without the leading '#'. Falls back to defaults on any error. */
export function parseState(hash: string): SceneState {
  let body = hash;
  if (body.startsWith('#')) {
    body = body.slice(1);
  }
  if (!body.startsWith(HASH_PREFIX)) {
    return defaultState();
  }
  try {
    return sanitizeState(JSON.parse(decodeURIComponent(body.slice(HASH_PREFIX.length))));
  } catch {
    return defaultState();
  }
}
