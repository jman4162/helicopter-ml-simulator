import { Control, HeliState } from '../physics/heli';
import { Quat, rotate as qrotate } from '../math/quaternion';
import { length } from '../math/vec3';

const DEG = 180 / Math.PI;

/** Aerospace 3-2-1 (yaw-pitch-roll) Euler angles, radians, from a body->world NED quaternion. */
const toEuler = (q: Quat): { roll: number; pitch: number; yaw: number } => {
  const { w, x, y, z } = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const sp = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
  const pitch = Math.asin(sp);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
};

const bar = (label: string, value: number): string => {
  // value in [-1, 1] -> a centered bar.
  const pct = (value * 50).toFixed(0);
  const left = value >= 0 ? 50 : 50 + Number(pct);
  const width = Math.abs(value * 50);
  return `<div class="bar"><span>${label}</span><div class="track"><div class="fill" style="left:${left}%;width:${width}%"></div></div><em>${value.toFixed(2)}</em></div>`;
};

export interface LearningInfo {
  iteration: number;
  totalIterations: number;
  rmse: number;
  naiveRmse: number;
  demoCount: number;
  posNoise: number;
  done: boolean;
}

export class Hud {
  private last = 0;
  constructor(private readonly el: HTMLElement) {}

  /** Render the system-identification panel. */
  showSysid(
    info: {
      coeffs: { name: string; fitted: number; truth: number }[];
      worstErrorPct: number;
      residual: number;
      predRmse: number;
      seconds: number;
    },
    fps: number,
  ): void {
    const rows = info.coeffs
      .map(
        (c) =>
          `<div class="row"><span>${c.name}</span><b>${c.fitted.toFixed(2)} <em style="color:var(--muted)">vs ${c.truth.toFixed(2)}</em></b></div>`,
      )
      .join('');
    this.el.innerHTML = `
      <div class="panel">
        <div class="title">SYSTEM IDENTIFICATION</div>
        <div class="row"><span>fit from</span><b>${info.seconds}s excitation flight</b></div>
        <div class="row"><span>worst coeff err</span><b class="on">${info.worstErrorPct.toFixed(1)}%</b></div>
        <div class="row"><span>1-step residual</span><b>${info.residual.toFixed(3)}</b></div>
        <div class="row"><span>predict err</span><b>${info.predRmse.toFixed(3)} m</b></div>
      </div>
      <div class="panel">
        <div class="title">FITTED vs TRUE</div>
        ${rows}
      </div>
      <div class="panel legend">
        <div class="title">HELD-OUT TEST FLIGHT</div>
        <div class="row"><span><i class="sw truth"></i> true model</span><b></b></div>
        <div class="row"><span><i class="sw est"></i> fitted prediction</span><b></b></div>
      </div>
      <div class="panel meta"><span>I new flight · M exit</span><span>${fps.toFixed(0)} fps</span></div>`;
  }

  /** Render the apprenticeship-learning panel (replaces flight panels in learning mode). */
  showLearning(info: LearningInfo, fps: number): void {
    const pct = info.naiveRmse > 0 ? (100 * (1 - info.rmse / info.naiveRmse)).toFixed(0) : '0';
    this.el.innerHTML = `
      <div class="panel">
        <div class="title">APPRENTICESHIP LEARNING</div>
        <div class="row"><span>demos</span><b>${info.demoCount} (noise ${info.posNoise.toFixed(2)})</b></div>
        <div class="row"><span>EM iteration</span><b class="on">${info.iteration} / ${info.totalIterations}${info.done ? ' ✓' : ''}</b></div>
        <div class="row"><span>recovery err</span><b>${info.rmse.toFixed(3)}</b></div>
        <div class="row"><span>naive average</span><b>${info.naiveRmse.toFixed(3)}</b></div>
        <div class="row"><span>error vs naive</span><b class="on">−${pct}%</b></div>
      </div>
      <div class="panel legend">
        <div class="title">LEGEND</div>
        <div class="row"><span><i class="sw demo"></i> demonstrations</span><b>${info.demoCount}</b></div>
        <div class="row"><span><i class="sw truth"></i> ground truth</span><b></b></div>
        <div class="row"><span><i class="sw est"></i> recovered</span><b></b></div>
      </div>
      <div class="panel meta"><span>L new demos · M exit</span><span>${fps.toFixed(0)} fps</span></div>`;
  }

  update(
    state: HeliState,
    control: Control,
    cameraMode: string,
    fps: number,
    time: number,
    auto: { mode: string; label: string; trackingError?: number },
  ): void {
    if (time - this.last < 1 / 15) return; // throttle DOM updates to ~15 Hz
    this.last = time;

    const { roll, pitch, yaw } = toEuler(state.orientation);
    const altitude = -state.position.z;
    const airspeed = length(state.velBody);
    const worldVel = qrotate(state.orientation, state.velBody);
    const climbRate = -worldVel.z; // NED: up is -z
    const r = state.rateBody;

    this.el.innerHTML = `
      <div class="panel">
        <div class="title">FLIGHT</div>
        <div class="row"><span>alt</span><b>${altitude.toFixed(1)} m</b></div>
        <div class="row"><span>airspeed</span><b>${airspeed.toFixed(1)} m/s</b></div>
        <div class="row"><span>climb</span><b>${climbRate.toFixed(1)} m/s</b></div>
        <div class="row"><span>rotor</span><b>${state.rotorSpeed.toFixed(0)} rad/s</b></div>
      </div>
      <div class="panel">
        <div class="title">ATTITUDE</div>
        <div class="row"><span>roll φ</span><b>${(roll * DEG).toFixed(0)}°</b></div>
        <div class="row"><span>pitch θ</span><b>${(pitch * DEG).toFixed(0)}°</b></div>
        <div class="row"><span>yaw ψ</span><b>${(yaw * DEG).toFixed(0)}°</b></div>
        <div class="row"><span>rates pqr</span><b>${(r.x * DEG).toFixed(0)}, ${(r.y * DEG).toFixed(0)}, ${(r.z * DEG).toFixed(0)} °/s</b></div>
      </div>
      <div class="panel">
        <div class="title">CONTROLS</div>
        ${bar('roll  u₁', control.u1)}
        ${bar('pitch u₂', control.u2)}
        ${bar('yaw   u₃', control.u3)}
        ${bar('coll  u₄', control.u4)}
      </div>
      <div class="panel">
        <div class="title">AUTOPILOT</div>
        <div class="row"><span>mode</span><b class="${auto.mode === 'manual' ? '' : 'on'}">${auto.label}</b></div>
        ${auto.trackingError !== undefined ? `<div class="row"><span>track err</span><b>${auto.trackingError.toFixed(2)}</b></div>` : ''}
      </div>
      <div class="panel meta">
        <span>cam: ${cameraMode}</span><span>${fps.toFixed(0)} fps</span>
      </div>`;
  }
}
