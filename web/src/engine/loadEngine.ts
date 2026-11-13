import { MockEngine } from './mockEngine';
import type { PhysicsWorld } from './types';

export type EngineBackend = 'wasm' | 'mock';

export interface EngineHandle {
  backend: EngineBackend;
  /** Why the mock is active (absent when wasm loaded). */
  reason?: string;
  createWorld(): PhysicsWorld;
}

/**
 * Shape expected from the Emscripten module (MODULARIZE=1, EXPORT_ES6=1):
 *   export default function createEngineModule(opts?): Promise<Module>
 * where Module.PhysicsWorld is an embind class implementing PhysicsWorld
 * (constructed with `new Module.PhysicsWorld()`).
 */
interface WasmModule {
  PhysicsWorld: new () => PhysicsWorld & { delete?: () => void };
}
type WasmFactory = (opts?: { locateFile?: (path: string) => string }) => Promise<WasmModule>;

export const WASM_BASE = '/wasm/';
export const WASM_ENTRY = `${WASM_BASE}engine.js`;

function wrapWasmWorld(raw: PhysicsWorld & { delete?: () => void }): PhysicsWorld {
  const destroy = raw.destroy;
  raw.destroy = () => {
    if (typeof destroy === 'function') {
      destroy.call(raw);
    }
    if (typeof raw.delete === 'function') {
      raw.delete();
    }
  };
  return raw;
}

export async function loadEngine(): Promise<EngineHandle> {
  try {
    const mod = (await import(/* webpackIgnore: true */ /* turbopackIgnore: true */ WASM_ENTRY)) as {
      default?: WasmFactory;
    };
    if (typeof mod.default !== 'function') {
      throw new Error('engine.js has no default factory export');
    }
    const wasm = await mod.default({ locateFile: (p) => WASM_BASE + p });
    if (typeof wasm.PhysicsWorld !== 'function') {
      throw new Error('module does not export PhysicsWorld');
    }
    return { backend: 'wasm', createWorld: () => wrapWasmWorld(new wasm.PhysicsWorld()) };
  } catch (err) {
    let reason = String(err);
    if (err instanceof Error) {
      reason = err.message;
    }
    return { backend: 'mock', reason, createWorld: () => new MockEngine() };
  }
}
