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

export class Hud {
  private last = 0;
  constructor(private readonly el: HTMLElement) {}

  update(state: HeliState, control: Control, cameraMode: string, fps: number, time: number): void {
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
      <div class="panel meta">
        <span>cam: ${cameraMode}</span><span>${fps.toFixed(0)} fps</span>
      </div>`;
  }
}
