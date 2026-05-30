import * as THREE from 'three';
import { Vec3 } from '../math/vec3';
import { Quat } from '../math/quaternion';

/**
 * The physics runs in a NED world frame (x north/forward, y east/right, z down).
 * Three.js is y-up. This module holds the single proper rotation `M` that maps
 * NED world coordinates into Three's frame, so the conversion lives in one place.
 *
 *   up_three   = -z_ned       (NED down -> Three up)
 *   right_three=  y_ned        (NED east -> Three +x)
 *   fwd(north) =  x_ned -> -z_three   (into the screen)
 *
 * det(M) = +1, so orientations are preserved (no mirroring).
 */
export const NED_TO_THREE = new THREE.Matrix4().set(
  0, 1, 0, 0,
  0, 0, -1, 0,
  -1, 0, 0, 0,
  0, 0, 0, 1,
);

const Q_NED_TO_THREE = new THREE.Quaternion().setFromRotationMatrix(NED_TO_THREE);

/** Convert a NED world position to a Three.js position. */
export const toThreePosition = (p: Vec3, out = new THREE.Vector3()): THREE.Vector3 =>
  out.set(p.x, p.y, p.z).applyMatrix4(NED_TO_THREE);

/**
 * Convert a body->world(NED) quaternion to a body->Three quaternion.
 * body->three = (world_ned->three) ∘ (body->world_ned).
 */
export const toThreeQuaternion = (q: Quat, out = new THREE.Quaternion()): THREE.Quaternion => {
  out.set(q.x, q.y, q.z, q.w); // THREE order is (x, y, z, w)
  return out.premultiply(Q_NED_TO_THREE);
};
