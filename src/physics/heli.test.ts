import { describe, it, expect } from 'vitest';
import {
  accelerations,
  defaultParams,
  hoverControl,
  initialState,
  step,
  zeroControl,
  G,
} from './heli';
import { norm } from '../math/quaternion';
import { length, vec3 } from '../math/vec3';

const P = defaultParams();

describe('helicopter dynamics (Eq. 1)', () => {
  it('hover trim: hover collective cancels gravity (zero acceleration)', () => {
    const s = initialState(P);
    const { lin, ang } = accelerations(P, s, hoverControl(P));
    expect(length(lin)).toBeLessThan(1e-9);
    expect(length(ang)).toBeLessThan(1e-9);
  });

  it('hover trim is a fixed point over time', () => {
    let s = initialState(P);
    const c = hoverControl(P);
    for (let i = 0; i < 600; i++) s = step(P, s, c, 1 / 100);
    expect(length(s.position)).toBeLessThan(1e-6);
    expect(length(s.velBody)).toBeLessThan(1e-6);
  });

  it('zero collective from level flight free-falls (gravity = +z down)', () => {
    const s = initialState(P);
    const { lin } = accelerations(P, s, zeroControl());
    expect(lin.z).toBeCloseTo(G, 6); // ẇ ≈ +g downward
    // Integrate briefly: should gain downward velocity and lose altitude.
    let st = initialState(P);
    for (let i = 0; i < 50; i++) st = step(P, st, zeroControl(), 1 / 100);
    expect(st.velBody.z).toBeGreaterThan(0); // moving down (+z)
    expect(st.position.z).toBeGreaterThan(0); // descended (+z is down)
  });

  it('full collective from level flight climbs', () => {
    let st = initialState(P);
    const climb = { u1: 0, u2: 0, u3: 0, u4: 1 };
    for (let i = 0; i < 50; i++) st = step(P, st, climb, 1 / 100);
    expect(st.position.z).toBeLessThan(0); // -z is up: gained altitude
  });

  it('roll command drives body roll rate to its steady state p = -C1·u1/Bx', () => {
    let st = initialState(P);
    const roll = { u1: 1, u2: 0, u3: 0, u4: hoverControl(P).u4 };
    for (let i = 0; i < 500; i++) st = step(P, st, roll, 1 / 200);
    const pSteady = (-P.C1 * 1) / P.Bx; // = 6 rad/s
    expect(st.rateBody.x).toBeCloseTo(pSteady, 1);
  });

  it('gyroscopic coupling has the right sign (ṙ = p·q·(Ixx−Iyy)/Izz)', () => {
    const s = { ...initialState(P), rateBody: vec3(1, 1, 0) }; // p=q=1, r=0
    const { ang } = accelerations(P, s, zeroControl());
    // Ixx < Iyy ⇒ (Ixx−Iyy) < 0 ⇒ ṙ < 0 for p,q > 0.
    expect(ang.z).toBeLessThan(0);
    expect(ang.z).toBeCloseTo((P.Ixx - P.Iyy) / P.Izz, 6);
  });

  it('quaternion stays normalized after many steps with tumbling input', () => {
    let st = initialState(P);
    const tumble = { u1: 0.7, u2: -0.4, u3: 0.3, u4: 0.6 };
    for (let i = 0; i < 1000; i++) st = step(P, st, tumble, 1 / 100);
    expect(Math.abs(norm(st.orientation) - 1)).toBeLessThan(1e-6);
  });

  it('main-rotor governor relaxes toward target speed', () => {
    let st = { ...initialState(P), rotorSpeed: 0 };
    for (let i = 0; i < 500; i++) st = step(P, st, hoverControl(P), 1 / 100);
    expect(st.rotorSpeed).toBeCloseTo(P.rotorTarget, 0);
  });
});
