import { describe, it, expect } from 'vitest';
import { cross, dot, length, normalize, vec3 } from './vec3';
import {
  derivative,
  fromAxisAngle,
  identity,
  normalize as qnormalize,
  rotate,
  rotateInverse,
  toAxisAngle,
} from './quaternion';
import { rk4 } from './integrate';
import { inverse, matMul, matVec, solve, transpose } from './matrix';

const close = (a: number, b: number, eps = 1e-9) => expect(Math.abs(a - b)).toBeLessThan(eps);

describe('vec3', () => {
  it('cross product is right-handed', () => {
    const c = cross(vec3(1, 0, 0), vec3(0, 1, 0));
    expect(c).toEqual(vec3(0, 0, 1));
  });
  it('dot and length agree', () => {
    const v = vec3(3, 4, 0);
    close(length(v), 5);
    close(dot(v, v), 25);
  });
  it('normalize yields unit length', () => {
    close(length(normalize(vec3(2, -3, 6))), 1);
  });
});

describe('quaternion', () => {
  it('90° rotation about z maps x -> y', () => {
    const q = fromAxisAngle(vec3(0, 0, Math.PI / 2));
    const r = rotate(q, vec3(1, 0, 0));
    close(r.x, 0, 1e-9);
    close(r.y, 1, 1e-9);
    close(r.z, 0, 1e-9);
  });

  it('rotate then rotateInverse is identity', () => {
    const q = qnormalize(fromAxisAngle(vec3(0.3, -0.7, 1.1)));
    const v = vec3(1, 2, 3);
    const back = rotateInverse(q, rotate(q, v));
    close(back.x, 1);
    close(back.y, 2);
    close(back.z, 3);
  });

  it('fromAxisAngle / toAxisAngle round-trip', () => {
    const aa = vec3(0.2, -0.5, 0.9);
    const out = toAxisAngle(fromAxisAngle(aa));
    close(out.x, 0.2, 1e-9);
    close(out.y, -0.5, 1e-9);
    close(out.z, 0.9, 1e-9);
  });

  it('derivative integrates a constant body spin into the right angle', () => {
    // Spin about body z at 1 rad/s for 1 s via small Euler steps -> ~90°+ check magnitude.
    let q = identity();
    const omega = vec3(0, 0, 1);
    const dt = 1e-4;
    for (let i = 0; i < 10000; i++) {
      const d = derivative(q, omega);
      q = qnormalize({ w: q.w + d.w * dt, x: q.x + d.x * dt, y: q.y + d.y * dt, z: q.z + d.z * dt });
    }
    const aa = toAxisAngle(q);
    close(aa.z, 1, 1e-3); // 1 rad about z
  });
});

describe('integrate (rk4)', () => {
  it('solves dy/dt = y (exponential) accurately', () => {
    let y = [1];
    const dt = 0.01;
    for (let i = 0; i < 100; i++) y = rk4((s) => [s[0]], y, dt);
    close(y[0], Math.E, 1e-6);
  });

  it('conserves energy of a harmonic oscillator over one period', () => {
    // x'' = -x  ->  state [x, v], deriv [v, -x]. Energy = x^2 + v^2.
    let y = [1, 0];
    const dt = 1e-3;
    const steps = Math.round((2 * Math.PI) / dt);
    for (let i = 0; i < steps; i++) y = rk4((s) => [s[1], -s[0]], y, dt);
    // RK4 is not symplectic, so allow a small phase error after a full period.
    close(y[0], 1, 1e-3);
    close(y[1], 0, 1e-3);
  });
});

describe('matrix', () => {
  it('matMul matches by-hand result', () => {
    const a = [
      [1, 2],
      [3, 4],
    ];
    const b = [
      [5, 6],
      [7, 8],
    ];
    expect(matMul(a, b)).toEqual([
      [19, 22],
      [43, 50],
    ]);
  });

  it('solve recovers a known solution', () => {
    const A = [
      [2, 1],
      [1, 3],
    ];
    const x = solve(A, matVec(A, [1, -2]));
    close(x[0], 1, 1e-9);
    close(x[1], -2, 1e-9);
  });

  it('inverse times matrix is identity', () => {
    const A = [
      [4, 7, 2],
      [3, 6, 1],
      [2, 5, 3],
    ];
    const prod = matMul(A, inverse(A));
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++) close(prod[i][j], i === j ? 1 : 0, 1e-9);
  });

  it('transpose is involutive', () => {
    const A = [
      [1, 2, 3],
      [4, 5, 6],
    ];
    expect(transpose(transpose(A))).toEqual(A);
  });
});
