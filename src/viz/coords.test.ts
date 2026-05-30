import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { toThreePosition, toThreeQuaternion } from './coords';
import { vec3 } from '../math/vec3';
import { identity, fromAxisAngle } from '../math/quaternion';

const closeVec = (v: THREE.Vector3, x: number, y: number, z: number, eps = 1e-9) => {
  expect(Math.abs(v.x - x)).toBeLessThan(eps);
  expect(Math.abs(v.y - y)).toBeLessThan(eps);
  expect(Math.abs(v.z - z)).toBeLessThan(eps);
};

describe('NED <-> Three.js coordinate mapping', () => {
  it('maps NED axes to the intended Three axes', () => {
    closeVec(toThreePosition(vec3(0, 0, -1)), 0, 1, 0); // NED up (-z)   -> Three +y
    closeVec(toThreePosition(vec3(1, 0, 0)), 0, 0, -1); // NED forward(x) -> Three -z (into screen)
    closeVec(toThreePosition(vec3(0, 1, 0)), 1, 0, 0); // NED right (y)  -> Three +x
  });

  it('a level helicopter points its body-x forward (Three -z)', () => {
    const q3 = toThreeQuaternion(identity());
    const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q3); // body x
    closeVec(fwd, 0, 0, -1, 1e-9);
    const up = new THREE.Vector3(0, 0, -1).applyQuaternion(q3); // body up (-z)
    closeVec(up, 0, 1, 0, 1e-9);
  });

  it('a 90° yaw (about body z / NED down) turns forward toward Three +x', () => {
    // Yaw right by 90° about body z (down). Forward should swing to NED east -> Three +x.
    const q3 = toThreeQuaternion(fromAxisAngle(vec3(0, 0, Math.PI / 2)));
    const fwd = new THREE.Vector3(1, 0, 0).applyQuaternion(q3);
    closeVec(fwd, 1, 0, 0, 1e-9);
  });
});
