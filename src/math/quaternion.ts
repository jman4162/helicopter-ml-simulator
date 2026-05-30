import { Vec3, vec3 } from './vec3';

/**
 * Unit quaternion representing orientation. Convention: `q` rotates a vector
 * expressed in the BODY frame into the WORLD frame (body -> world). Stored as
 * (w, x, y, z) with w the scalar part.
 */
export interface Quat {
  w: number;
  x: number;
  y: number;
  z: number;
}

export const quat = (w = 1, x = 0, y = 0, z = 0): Quat => ({ w, x, y, z });

export const identity = (): Quat => ({ w: 1, x: 0, y: 0, z: 0 });

/** Hamilton product a ⊗ b. */
export const multiply = (a: Quat, b: Quat): Quat => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});

export const conjugate = (a: Quat): Quat => ({ w: a.w, x: -a.x, y: -a.y, z: -a.z });

export const norm = (a: Quat): number => Math.sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);

export const normalize = (a: Quat): Quat => {
  const n = norm(a);
  if (n < 1e-12) return identity();
  const inv = 1 / n;
  return { w: a.w * inv, x: a.x * inv, y: a.y * inv, z: a.z * inv };
};

/** Build a rotation quaternion from an axis-angle vector (axis * angle, radians). */
export const fromAxisAngle = (aa: Vec3): Quat => {
  const angle = Math.sqrt(aa.x * aa.x + aa.y * aa.y + aa.z * aa.z);
  if (angle < 1e-12) return identity();
  const half = angle / 2;
  const s = Math.sin(half) / angle;
  return { w: Math.cos(half), x: aa.x * s, y: aa.y * s, z: aa.z * s };
};

/**
 * Axis-angle vector (axis * angle) of a unit quaternion, with angle in (-π, π].
 * Used for the control error state, where a minimal 3-D rotation error is needed.
 */
export const toAxisAngle = (q: Quat): Vec3 => {
  const n = normalize(q);
  // Ensure shortest path (w >= 0).
  const w = n.w < 0 ? -n.w : n.w;
  const sign = n.w < 0 ? -1 : 1;
  const vlen = Math.sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
  if (vlen < 1e-12) return vec3(0, 0, 0);
  const angle = 2 * Math.atan2(vlen, w);
  const s = (sign * angle) / vlen;
  return vec3(n.x * s, n.y * s, n.z * s);
};

/** Rotate a body-frame vector into the world frame. */
export const rotate = (q: Quat, v: Vec3): Vec3 => {
  const p: Quat = { w: 0, x: v.x, y: v.y, z: v.z };
  const r = multiply(multiply(q, p), conjugate(q));
  return vec3(r.x, r.y, r.z);
};

/** Rotate a world-frame vector into the body frame. */
export const rotateInverse = (q: Quat, v: Vec3): Vec3 => {
  const p: Quat = { w: 0, x: v.x, y: v.y, z: v.z };
  const r = multiply(multiply(conjugate(q), p), q);
  return vec3(r.x, r.y, r.z);
};

/**
 * Time derivative q̇ given body-frame angular velocity ω = (p, q, r).
 * q̇ = ½ · q ⊗ (0, ω).
 */
export const derivative = (q: Quat, omegaBody: Vec3): Quat => {
  const wq: Quat = { w: 0, x: omegaBody.x, y: omegaBody.y, z: omegaBody.z };
  const d = multiply(q, wq);
  return { w: 0.5 * d.w, x: 0.5 * d.x, y: 0.5 * d.y, z: 0.5 * d.z };
};

/** Relative rotation taking `from` to `to`: r = to ⊗ from⁻¹ (world-frame error). */
export const delta = (from: Quat, to: Quat): Quat => multiply(to, conjugate(from));
