'use client';

import { useEffect, useMemo, useRef, useState } from 'react';
import { loadEngine } from '../engine/loadEngine';
import type { EngineHandle } from '../engine/loadEngine';
import { DEFAULT_OVERLAYS } from '../render/renderer';
import type { Overlays } from '../render/renderer';
import { createTelemetry } from '../render/telemetry';
import { applyArmJoints, armJointKey, armStructureKey, buildArm } from '../scenes/arm';
import { findGalleryScene, GALLERY } from '../scenes/gallery';
import { defaultLink, defaultState, LIMITS, MAX_LINKS, MIN_LINKS, parseState, serializeState } from '../scenes/sceneState';
import type { LinkParams, SceneState, TabId } from '../scenes/sceneState';
import { ArmEditor, NumberField } from './ArmEditor';
import { EnergyChart } from './EnergyChart';
import { Hud } from './Hud';
import { Viewport } from './Viewport';

const OVERLAY_LABELS: Record<keyof Overlays, string> = {
  jointAnchors: 'Joint anchors',
  contacts: 'Contacts',
  velocities: 'Velocity vectors',
  centerOfMass: 'Center of mass',
};

function tabClass(active: boolean): string {
  if (active) {
    return 'tab active';
  }
  return 'tab';
}

export function Demo() {
  const [state, setState] = useState<SceneState>(defaultState);
  const [hashReady, setHashReady] = useState(false);
  const [handle, setHandle] = useState<EngineHandle | null>(null);
  const [playing, setPlaying] = useState(true);
  const [resetNonce, setResetNonce] = useState(0);
  const [stepNonce, setStepNonce] = useState(0);
  const [overlays, setOverlays] = useState<Overlays>(DEFAULT_OVERLAYS);
  const telemetry = useRef(createTelemetry());

  useEffect(() => {
    // Hash is only readable on the client; hydrate after mount to keep SSR output stable.
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setState(parseState(window.location.hash));
    setHashReady(true);
    let cancelled = false;
    loadEngine().then((h) => {
      if (!cancelled) {
        setHandle(h);
      }
    });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (hashReady) {
      window.history.replaceState(null, '', `#${serializeState(state)}`);
    }
  }, [state, hashReady]);

  const patch = (p: Partial<SceneState>) => setState((s) => ({ ...s, ...p }));
  const setLinks = (links: LinkParams[]) => patch({ links });

  const gallery = findGalleryScene(state.tab);
  const armStructure = armStructureKey(state.links);
  const armJoints = armJointKey(state.links);

  const scene = useMemo(() => {
    if (gallery) {
      return {
        buildKey: gallery.id,
        liveKey: '',
        build: gallery.build,
        live: undefined,
      };
    }
    return {
      buildKey: `arm:${armStructure}`,
      liveKey: armJoints,
      build: (w: Parameters<typeof buildArm>[0]) => buildArm(w, state.links),
      live: (w: Parameters<typeof applyArmJoints>[0], b: Parameters<typeof applyArmJoints>[1]) =>
        applyArmJoints(w, b, state.links),
    };
    // state.links is captured through armStructure/armJoints keys.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [gallery, armStructure, armJoints]);

  const showWasmWarning = gallery && gallery.needsCollisions && handle && handle.backend === 'mock';

  const tabs: { id: TabId; title: string }[] = [{ id: 'arm', title: 'Robotic arm' }];
  GALLERY.forEach((g) => tabs.push({ id: g.id as TabId, title: g.title }));

  return (
    <div className="layout">
      <aside className="sidebar">
        <h1>Physics Engine</h1>
        {handle && handle.backend === 'mock' && (
          <p className="note" title={handle.reason}>
            Using the JS mock engine (no WASM build found).
          </p>
        )}
        <section>
          <h3>Simulation</h3>
          <div className="row">
            <button onClick={() => setPlaying((p) => !p)}>{playing && 'Pause'}{!playing && 'Play'}</button>
            <button
              onClick={() => {
                setPlaying(false);
                setStepNonce((n) => n + 1);
              }}
            >
              Step
            </button>
            <button onClick={() => setResetNonce((n) => n + 1)}>Reset</button>
          </div>
          <NumberField label="Gravity" unit=" m/s²" value={state.gravity} {...LIMITS.gravity} step={0.1} onChange={(v) => patch({ gravity: v })} />
          <NumberField label="Time scale" unit="x" value={state.timeScale} {...LIMITS.timeScale} step={0.05} onChange={(v) => patch({ timeScale: v })} />
          <NumberField label="Hand strength" unit="x" value={state.handStrength} {...LIMITS.handStrength} step={0.05} onChange={(v) => patch({ handStrength: v })} />
        </section>
        <section>
          <h3>Debug overlays</h3>
          {(Object.keys(OVERLAY_LABELS) as (keyof Overlays)[]).map((k) => (
            <label className="check" key={k}>
              <input type="checkbox" checked={overlays[k]} onChange={(e) => setOverlays({ ...overlays, [k]: e.target.checked })} />
              {OVERLAY_LABELS[k]}
            </label>
          ))}
        </section>
        {!gallery && (
          <ArmEditor
            links={state.links}
            onChange={setLinks}
            onAdd={() => {
              if (state.links.length < MAX_LINKS) {
                setLinks([...state.links, defaultLink()]);
              }
            }}
            onRemove={() => {
              if (state.links.length > MIN_LINKS) {
                setLinks(state.links.slice(0, -1));
              }
            }}
          />
        )}
      </aside>
      <main className="main">
        <nav className="tabs">
          {tabs.map((t) => (
            <button key={t.id} className={tabClass(t.id === state.tab)} onClick={() => patch({ tab: t.id })}>
              {t.title}
            </button>
          ))}
        </nav>
        <div className="stage">
          {handle && (
            <Viewport
              handle={handle}
              buildKey={scene.buildKey}
              build={scene.build}
              liveKey={scene.liveKey}
              live={scene.live}
              cameraKey={state.tab}
              gravity={state.gravity}
              timeScale={state.timeScale}
              handStrength={state.handStrength}
              playing={playing}
              resetNonce={resetNonce}
              stepNonce={stepNonce}
              overlays={overlays}
              telemetry={telemetry}
            />
          )}
          <Hud telemetry={telemetry} handle={handle} />
          {showWasmWarning && <div className="banner">Needs WASM engine: collisions are not simulated by the mock.</div>}
        </div>
        <EnergyChart telemetry={telemetry} />
      </main>
    </div>
  );
}
