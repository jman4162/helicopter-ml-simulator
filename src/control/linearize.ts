import { Control, HeliParams, HeliState, step } from '../physics/heli';
import { Matrix, zeros } from '../math/matrix';
import { vec3 } from '../math/vec3';
import { conjugate, fromAxisAngle, multiply, toAxisAngle } from '../math/quaternion';

/**
 * The controller works in a minimal 12-D error (tangent) state, since the 4-D
 * quaternion has only 3 rotational DOF. Ordering:
 *   [ 0:3 ] position error (world frame)
 *   [ 3:6 ] attitude error, axis-angle, in the reference body frame (q = ref⁻¹ ⊗ s)
 *   [ 6:9 ] body-velocity error (u, v, w)
 *   [ 9:12] body-rate error (p, q, r)
 * This mirrors the paper's error state (§6.2.1), which uses an axis-angle
 * attitude error because it linearizes the rotation far more faithfully than
 * differencing quaternions or Euler angles.
 */
export const ERR_DIM = 12;
export const CTRL_DIM = 4;

/** Apply a tangent perturbation δ to a state (the ⊞ "box-plus" operator). */
export const boxplus = (s: HeliState, d: number[]): HeliState => ({
  position: vec3(s.position.x + d[0], s.position.y + d[1], s.position.z + d[2]),
  orientation: multiply(s.orientation, fromAxisAngle(vec3(d[3], d[4], d[5]))),
  velBody: vec3(s.velBody.x + d[6], s.velBody.y + d[7], s.velBody.z + d[8]),
  rateBody: vec3(s.rateBody.x + d[9], s.rateBody.y + d[10], s.rateBody.z + d[11]),
  rotorSpeed: s.rotorSpeed,
});

/** Tangent difference s ⊟ ref, inverse of boxplus. */
export const boxminus = (s: HeliState, ref: HeliState): number[] => {
  const att = toAxisAngle(multiply(conjugate(ref.orientation), s.orientation));
  return [
    s.position.x - ref.position.x,
    s.position.y - ref.position.y,
    s.position.z - ref.position.z,
    att.x,
    att.y,
    att.z,
    s.velBody.x - ref.velBody.x,
    s.velBody.y - ref.velBody.y,
    s.velBody.z - ref.velBody.z,
    s.rateBody.x - ref.rateBody.x,
    s.rateBody.y - ref.rateBody.y,
    s.rateBody.z - ref.rateBody.z,
  ];
};

const ctrlToArray = (c: Control): number[] => [c.u1, c.u2, c.u3, c.u4];
const arrayToCtrl = (a: number[]): Control => ({ u1: a[0], u2: a[1], u3: a[2], u4: a[3] });

/**
 * Finite-difference linearization of the discrete dynamics δ_{t+1} = A δ_t + B δu_t
 * about a reference (sRef, uRef) over one control step `dt`. Central differences
 * in the tangent space; A is 12×12, B is 12×4.
 */
export const linearizeDiscrete = (
  params: HeliParams,
  sRef: HeliState,
  uRef: Control,
  dt: number,
  eps = 1e-4,
): { A: Matrix; B: Matrix } => {
  const sNext = step(params, sRef, uRef, dt);
  const A = zeros(ERR_DIM, ERR_DIM);
  const B = zeros(ERR_DIM, CTRL_DIM);

  const e = new Array<number>(ERR_DIM).fill(0);
  for (let j = 0; j < ERR_DIM; j++) {
    e[j] = eps;
    const fp = boxminus(step(params, boxplus(sRef, e), uRef, dt), sNext);
    e[j] = -eps;
    const fm = boxminus(step(params, boxplus(sRef, e), uRef, dt), sNext);
    e[j] = 0;
    for (let i = 0; i < ERR_DIM; i++) A[i][j] = (fp[i] - fm[i]) / (2 * eps);
  }

  const u = ctrlToArray(uRef);
  for (let k = 0; k < CTRL_DIM; k++) {
    const up = [...u];
    up[k] += eps;
    const um = [...u];
    um[k] -= eps;
    const fp = boxminus(step(params, sRef, arrayToCtrl(up), dt), sNext);
    const fm = boxminus(step(params, sRef, arrayToCtrl(um), dt), sNext);
    for (let i = 0; i < ERR_DIM; i++) B[i][k] = (fp[i] - fm[i]) / (2 * eps);
  }

  return { A, B };
};
