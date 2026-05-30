import { Control, HeliParams, hoverCollective } from '../physics/heli';
import { ManeuverId } from '../control/maneuvers';

const MANEUVER_KEYS: Record<string, ManeuverId> = {
  '1': 'forward',
  '2': 'square',
  '3': 'flip',
  '4': 'loop',
};

const STICK_KEYS = ['w', 's', 'a', 'd', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'];

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
  private pendingHover = false;
  private pendingGust = false;
  private pendingManual = false;
  private pendingLearning = false;
  private pendingManeuver: ManeuverId | null = null;

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
      if (k === 'h') this.pendingHover = true;
      if (k === 'g') this.pendingGust = true;
      if (k === 'm') this.pendingManual = true;
      if (k === 'l') this.pendingLearning = true;
      if (MANEUVER_KEYS[k]) this.pendingManeuver = MANEUVER_KEYS[k];
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

  /** Sync the held collective (e.g. to the autopilot output on disengage, to avoid a jump). */
  setCollective(value: number): void {
    this.collective = Math.min(1, Math.max(-1, value));
  }

  /** True while the pilot is touching the cyclic/collective/pedals (used to drop autopilot). */
  manualStickActive(): boolean {
    return STICK_KEYS.some((k) => this.keys.has(k));
  }

  consumeHover(): boolean {
    const v = this.pendingHover;
    this.pendingHover = false;
    return v;
  }

  consumeGust(): boolean {
    const v = this.pendingGust;
    this.pendingGust = false;
    return v;
  }

  consumeManual(): boolean {
    const v = this.pendingManual;
    this.pendingManual = false;
    return v;
  }

  consumeManeuver(): ManeuverId | null {
    const v = this.pendingManeuver;
    this.pendingManeuver = null;
    return v;
  }

  consumeLearning(): boolean {
    const v = this.pendingLearning;
    this.pendingLearning = false;
    return v;
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
