import { Quat, derivative as qDerivative, normalize as qNormalize, quat, rotate, rotateInverse } from '../math/quaternion';
import { Vec3, vec3 } from '../math/vec3';
import { rk4, State } from '../math/integrate';

/**
 * Rigid-body helicopter dynamics from Abbeel, Coates & Ng (IJRR 2010), Eq. (1).
 *
 * Frames (paper convention):
 *   - Body frame: x forward, y right, z down (attached to the helicopter).
 *   - World frame: NED-like, z down, gravity = (0, 0, +g).
 * The Three.js viz layer converts to its own y-up frame; the physics stays
 * faithful to the paper so the equations read exactly as published.
 *
 * Accelerations (body frame), with (u,v,w) linear vel, (p,q,r) angular rate,
 * (gx,gy,gz) gravity in body frame, and four control inputs:
 *   u1 lateral cyclic (roll), u2 longitudinal cyclic (pitch),
 *   u3 tail-rotor yaw-rate, u4 main-rotor collective (thrust).
 *
 *   u̇ = v·r − w·q + gx + Ax·u
 *   v̇ = w·p − u·r + gy + Ay·v + D0
 *   ẇ = u·q − v·p + gz + Az·w + C4·u4 + D4
 *   ṗ = q·r·(Iyy−Izz)/Ixx + Bx·p + C1·u1 + D1
 *   q̇ = p·r·(Izz−Ixx)/Iyy + By·q + C2·u2 + D2
 *   ṙ = p·q·(Ixx−Iyy)/Izz + Bz·r + C3·u3 + D3
 *
 * The A/B coefficients are (negative) drag/damping, C are control effectiveness,
 * D are biases. In this model angular rates relax to a control-set steady state
 * (ṗ≈0 ⇒ p ≈ −C1·u1/Bx), matching the paper's first-order-rate observation.
 */

export const G = 9.81;

export interface HeliParams {
  /** Linear drag (body axes), negative. */
  Ax: number;
  Ay: number;
  Az: number;
  /** Angular damping, negative. */
  Bx: number;
  By: number;
  Bz: number;
  /** Control effectiveness: roll, pitch, yaw, collective. */
  C1: number;
  C2: number;
  C3: number;
  C4: number;
  /** Bias terms. */
  D0: number;
  D1: number;
  D2: number;
  D3: number;
  D4: number;
  /** Moments of inertia (only their ratios enter the gyroscopic coupling). */
  Ixx: number;
  Iyy: number;
  Izz: number;
  /** Main-rotor governor: target speed (rad/s) and relaxation rate. */
  rotorTarget: number;
  rotorGain: number;
}

/**
 * Plausible aerobatic-class coefficients (XCell-Tempest-ish). Chosen so that:
 *   - hover collective u4 ≈ 0.40 (gz + C4·u4 = 0 ⇒ u4 = g/2.5g),
 *   - max collective thrust ≈ 2.5 g (supports flips / inverted hover),
 *   - steady-state rates at full stick ≈ roll 6, pitch 4.4, yaw 3 rad/s.
 * Not identified from real flight logs — Phase 4 (system ID) fits these from data.
 */
export const defaultParams = (): HeliParams => ({
  Ax: -0.1,
  Ay: -0.3,
  Az: -0.3,
  Bx: -5,
  By: -5,
  Bz: -5,
  C1: 30,
  C2: 22,
  C3: 15,
  C4: -2.5 * G,
  D0: 0,
  D1: 0,
  D2: 0,
  D3: 0,
  D4: 0,
  Ixx: 0.18,
  Iyy: 0.34,
  Izz: 0.28,
  rotorTarget: 160,
  rotorGain: 2,
});

/** Four-dimensional control input, each nominally in [-1, 1]. */
export interface Control {
  /** Lateral cyclic — roll. */
  u1: number;
  /** Longitudinal cyclic — pitch. */
  u2: number;
  /** Tail rotor — yaw rate. */
  u3: number;
  /** Main rotor — collective / thrust. */
  u4: number;
}

export const zeroControl = (): Control => ({ u1: 0, u2: 0, u3: 0, u4: 0 });

/** Structured helicopter state. */
export interface HeliState {
  /** Position in the world (NED) frame. */
  position: Vec3;
  /** Orientation, body -> world. */
  orientation: Quat;
  /** Linear velocity in the body frame (u, v, w). */
  velBody: Vec3;
  /** Angular velocity in the body frame (p, q, r). */
  rateBody: Vec3;
  /** Main-rotor speed (rad/s). */
  rotorSpeed: number;
}

/** Collective that holds a level hover for the given params (gz + C4·u4 = 0). */
export const hoverCollective = (p: HeliParams): number => -G / p.C4;

export const hoverControl = (p: HeliParams): Control => ({
  u1: 0,
  u2: 0,
  u3: 0,
  u4: hoverCollective(p),
});

export const initialState = (p: HeliParams): HeliState => ({
  position: vec3(0, 0, 0),
  orientation: quat(1, 0, 0, 0),
  velBody: vec3(0, 0, 0),
  rateBody: vec3(0, 0, 0),
  rotorSpeed: p.rotorTarget,
});

const clamp = (x: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, x));

/** Clamp each control channel into its valid range. */
export const clampControl = (c: Control): Control => ({
  u1: clamp(c.u1, -1, 1),
  u2: clamp(c.u2, -1, 1),
  u3: clamp(c.u3, -1, 1),
  u4: clamp(c.u4, -1, 1),
});

/**
 * Body-frame accelerations (u̇, v̇, ẇ, ṗ, q̇, ṙ) for a state + control, per Eq. 1.
 * Exposed separately because it is handy for the HUD and for unit tests.
 */
export const accelerations = (
  p: HeliParams,
  s: HeliState,
  c: Control,
): { lin: Vec3; ang: Vec3 } => {
  const { x: u, y: v, z: w } = s.velBody;
  const { x: pr, y: qr, z: rr } = s.rateBody; // body rates p, q, r
  const g = rotateInverse(s.orientation, vec3(0, 0, G)); // gravity in body frame

  const lin = vec3(
    v * rr - w * qr + g.x + p.Ax * u,
    w * pr - u * rr + g.y + p.Ay * v + p.D0,
    u * qr - v * pr + g.z + p.Az * w + p.C4 * c.u4 + p.D4,
  );

  const ang = vec3(
    qr * rr * ((p.Iyy - p.Izz) / p.Ixx) + p.Bx * pr + p.C1 * c.u1 + p.D1,
    pr * rr * ((p.Izz - p.Ixx) / p.Iyy) + p.By * qr + p.C2 * c.u2 + p.D2,
    pr * qr * ((p.Ixx - p.Iyy) / p.Izz) + p.Bz * rr + p.C3 * c.u3 + p.D3,
  );

  return { lin, ang };
};

// --- Flat <-> structured state, for the generic RK4 integrator. ---------------

const STATE_DIM = 14;

const toArray = (s: HeliState): State => [
  s.position.x, s.position.y, s.position.z,
  s.orientation.w, s.orientation.x, s.orientation.y, s.orientation.z,
  s.velBody.x, s.velBody.y, s.velBody.z,
  s.rateBody.x, s.rateBody.y, s.rateBody.z,
  s.rotorSpeed,
];

const fromArray = (a: State): HeliState => ({
  position: vec3(a[0], a[1], a[2]),
  orientation: { w: a[3], x: a[4], y: a[5], z: a[6] },
  velBody: vec3(a[7], a[8], a[9]),
  rateBody: vec3(a[10], a[11], a[12]),
  rotorSpeed: a[13],
});

/**
 * Advance one step with RK4. `disturbance`, if given, adds body-frame linear and
 * angular acceleration (e.g. a wind gust) — used later to demo MPC recovery.
 */
export const step = (
  p: HeliParams,
  s: HeliState,
  c: Control,
  dt: number,
  disturbance?: { lin?: Vec3; ang?: Vec3 },
): HeliState => {
  const u = clampControl(c);

  const deriv = (y: State): State => {
    const st = fromArray(y);
    const { lin, ang } = accelerations(p, st, u);
    const dl = disturbance?.lin ?? vec3();
    const da = disturbance?.ang ?? vec3();
    const posDot = rotate(st.orientation, st.velBody); // body vel -> world
    const qDot = qDerivative(st.orientation, st.rateBody);
    const out = new Array<number>(STATE_DIM);
    out[0] = posDot.x; out[1] = posDot.y; out[2] = posDot.z;
    out[3] = qDot.w; out[4] = qDot.x; out[5] = qDot.y; out[6] = qDot.z;
    out[7] = lin.x + dl.x; out[8] = lin.y + dl.y; out[9] = lin.z + dl.z;
    out[10] = ang.x + da.x; out[11] = ang.y + da.y; out[12] = ang.z + da.z;
    out[13] = -p.rotorGain * (st.rotorSpeed - p.rotorTarget);
    return out;
  };

  const next = fromArray(rk4(deriv, toArray(s), dt));
  next.orientation = qNormalize(next.orientation); // keep unit quaternion
  return next;
};
