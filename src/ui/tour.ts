/**
 * Guided tour: an ordered set of lessons that ties the three pillars of the
 * paper (physics, control, learning) into one narrative. Each step shows a short
 * explanation and the relevant equation, and auto-drives the matching simulator
 * mode so the idea is visible while it is described.
 */

export type TourMode = 'manual' | 'hover' | 'gust' | 'flip' | 'learning' | 'sysid' | 'airshow';

export interface TourStep {
  tag: string;
  title: string;
  body: string;
  equation?: string;
  mode: TourMode;
}

export const TOUR: TourStep[] = [
  {
    tag: 'intro',
    title: 'Autonomous Helicopter Aerobatics',
    body: 'A simulator of Abbeel, Coates &amp; Ng (2010). Three ideas, in order: model the <b>physics</b>, <b>control</b> it, and <b>learn</b> from imperfect demonstrations. Press <b>N</b> to step through; <b>T</b> leaves the tour.',
    mode: 'manual',
  },
  {
    tag: 'physics',
    title: '1 · The physics',
    body: 'A rigid body with four controls — lateral &amp; longitudinal cyclic (roll/pitch), tail rotor (yaw), and main-rotor collective (thrust). The model predicts body-frame accelerations. Fly it: <b>W/S</b> collective, <b>arrows</b> cyclic, <b>A/D</b> yaw.',
    equation: 'ṗ = (I<sub>yy</sub>−I<sub>zz</sub>)/I<sub>xx</sub>·qr + B<sub>x</sub>p + C₁u₁ &nbsp;·&nbsp; ẇ = uq−vp + g<sub>z</sub> + C₄u₄',
    mode: 'manual',
  },
  {
    tag: 'hover',
    title: '2 · Control — hold a hover',
    body: 'Linearize the dynamics at the hover trim and solve a Linear-Quadratic Regulator: a feedback law that drives any deviation back to the setpoint (the amber marker).',
    equation: 'u = −K·e &nbsp;·&nbsp; K minimizes Σ (eᵀQe + uᵀRu)',
    mode: 'hover',
  },
  {
    tag: 'wind',
    title: '2 · Control — reject disturbances',
    body: 'A wind gust knocks the helicopter off its setpoint; the same LQR feedback rejects it and returns to hover. Watch the tracking error spike and decay.',
    equation: 'e = x − x*  (error state; attitude error as an axis-angle vector)',
    mode: 'gust',
  },
  {
    tag: 'aerobatics',
    title: '2 · Control — aerobatics',
    body: 'For an aggressive maneuver, a <i>time-varying</i> LQR tracks a reference trajectory, re-stabilizing all the way through an in-place flip — even while inverted.',
    equation: 'K<sub>t</sub> from a backward Riccati pass along the reference',
    mode: 'flip',
  },
  {
    tag: 'learning',
    title: '3 · Learn the intended path',
    body: 'Real demonstrations are noisy and <b>time-warped</b>, and naive averaging blurs them. Align them (dynamic time warping) and smooth (Kalman) to recover the clean intended trajectory — the paper&#39;s key idea.',
    equation: 'y<sup>k</sup><sub>j</sub> = z<sub>τ</sub> + δ<sup>k</sup> + noise &nbsp; (a demo = warped, drifted, noisy truth)',
    mode: 'learning',
  },
  {
    tag: 'sysid',
    title: '3 · Learn the dynamics',
    body: 'The coefficients of the dynamics model are <i>linear</i> in the logged data, so least squares recovers them from a flight log. The fitted model (amber) then predicts a new flight (green).',
    equation: 'θ = (ΦᵀΦ)⁻¹Φᵀy',
    mode: 'sysid',
  },
  {
    tag: 'airshow',
    title: 'Putting it together',
    body: 'The autopilot now flies a full sequence on its own — forward flight, a box, a flip, a loop. Press <b>N</b> to restart the tour, or <b>T</b> to leave and fly it yourself.',
    mode: 'airshow',
  },
];
