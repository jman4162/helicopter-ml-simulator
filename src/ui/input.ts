import { Control, HeliParams, hoverCollective } from '../physics/heli';

/**
 * Manual flight input. Cyclic and yaw are momentary (spring back to center when
 * keys release, like a real RC stick); collective is a held throttle position
 * adjusted by W/S. A connected gamepad (mode-2 RC layout) overrides the keyboard
 * whenever its sticks are deflected.
 */
export class InputController {
  private readonly keys = new Set<string>();
  private collective: number;
  resetRequested = false;
  cameraCycleRequested = false;

  constructor(private readonly params: HeliParams) {
    this.collective = hoverCollective(params);
    window.addEventListener('keydown', (e) => this.onKey(e, true));
    window.addEventListener('keyup', (e) => this.onKey(e, false));
  }

  private onKey(e: KeyboardEvent, down: boolean): void {
    const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
    if (down) {
      if (k === 'r') this.resetRequested = true;
      if (k === 'c') this.cameraCycleRequested = true;
    }
    // Track movement keys; prevent page scroll on arrows/space.
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', ' '].includes(k)) e.preventDefault();
    if (down) this.keys.add(k);
    else this.keys.delete(k);
  }

  /** Reset the collective back to hover (called on simulation reset). */
  resetThrottle(): void {
    this.collective = hoverCollective(this.params);
  }

  private gamepad(): Gamepad | null {
    const pads = navigator.getGamepads?.() ?? [];
    for (const p of pads) if (p) return p;
    return null;
  }

  /** Read inputs for this frame. `dt` advances the held collective from W/S. */
  poll(dt: number): Control {
    const k = this.keys;
    // Collective: W up, S down, ~0.6 units/sec, clamped.
    if (k.has('w')) this.collective += 0.6 * dt;
    if (k.has('s')) this.collective -= 0.6 * dt;
    this.collective = Math.min(1, Math.max(-1, this.collective));

    let u1 = (k.has('ArrowRight') ? 1 : 0) - (k.has('ArrowLeft') ? 1 : 0);
    let u2 = (k.has('ArrowUp') ? 1 : 0) - (k.has('ArrowDown') ? 1 : 0);
    let u3 = (k.has('d') ? 1 : 0) - (k.has('a') ? 1 : 0);
    let u4 = this.collective;

    const pad = this.gamepad();
    if (pad) {
      const dead = (v: number) => (Math.abs(v) < 0.08 ? 0 : v);
      const lx = dead(pad.axes[0] ?? 0);
      const ly = dead(pad.axes[1] ?? 0);
      const rx = dead(pad.axes[2] ?? 0);
      const ry = dead(pad.axes[3] ?? 0);
      if (lx) u3 = lx; // left stick X -> yaw
      if (ly) {
        this.collective = -ly; // left stick Y -> collective (absolute)
        u4 = this.collective;
      }
      if (rx) u1 = rx; // right stick X -> roll
      if (ry) u2 = -ry; // right stick Y -> pitch (push forward = nose down/forward)
    }

    return { u1, u2, u3, u4 };
  }

  consumeReset(): boolean {
    const r = this.resetRequested;
    this.resetRequested = false;
    return r;
  }

  consumeCameraCycle(): boolean {
    const c = this.cameraCycleRequested;
    this.cameraCycleRequested = false;
    return c;
  }
}
