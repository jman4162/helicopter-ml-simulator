import { describe, it, expect } from 'vitest';
import { defaultParams } from '../physics/heli';
import { coefficientError, generateExcitation, identify, predictionError } from './sysid';

const P = defaultParams();
const DT = 1 / 500; // fine step keeps the central-difference accelerations accurate

describe('system identification (§3.3)', () => {
  it('recovers the true coefficients from clean excitation data', () => {
    const log = generateExcitation(P, DT, 12, 1);
    const { params, residual } = identify(log, P);

    // Key effectiveness/damping coefficients recovered tightly.
    expect(params.C1).toBeCloseTo(P.C1, 0);
    expect(params.C2).toBeCloseTo(P.C2, 0);
    expect(params.C4).toBeCloseTo(P.C4, 0);
    expect(Math.abs(params.Bx - P.Bx)).toBeLessThan(0.1);
    expect(Math.abs(params.Az - P.Az)).toBeLessThan(0.05);
    expect(coefficientError(params, P)).toBeLessThan(0.03); // <3% worst-case
    expect(residual).toBeLessThan(0.04);
  });

  it('recovers near-zero bias terms (true D = 0)', () => {
    const log = generateExcitation(P, DT, 12, 2);
    const { params } = identify(log, P);
    for (const d of [params.D0, params.D1, params.D2, params.D3, params.D4]) {
      expect(Math.abs(d)).toBeLessThan(0.2);
    }
  });

  it('stays usable under sensor noise (least squares averages it out)', () => {
    const log = generateExcitation(P, DT, 16, 3);
    const { params } = identify(log, P, 0.01); // ~1 cm/s, 0.01 rad/s sensor noise
    // Differentiation amplifies noise per-sample, but regression over thousands
    // of samples still recovers the coefficients within a useful margin.
    expect(coefficientError(params, P)).toBeLessThan(0.4);
  });

  it('fitted model predicts the true flight (simulation accuracy)', () => {
    const log = generateExcitation(P, DT, 10, 4);
    const { params } = identify(log, P);
    const { rmse } = predictionError(log, params);
    // Open-loop multi-step prediction stays close over the whole flight.
    expect(rmse).toBeLessThan(0.3);
  });
});
