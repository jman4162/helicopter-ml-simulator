import { Control, defaultParams, initialState, step, HeliState, zeroControl } from './physics/heli';
import { HeliScene } from './viz/scene';
import { InputController } from './ui/input';
import { Hud } from './ui/hud';
import { HoverController, TrajectoryController } from './control/autopilot';
import { boxminus } from './control/linearize';
import { buildManeuver, Reference } from './control/maneuvers';
import { vec3, Vec3 } from './math/vec3';
import { rotate, rotateInverse } from './math/quaternion';
import { generateDemos, defaultDemoOptions } from './learning/synthetic';
import { learnTrajectory, defaultLearnOptions, LearnResult } from './learning/trajectory';
import { airshowPath, toXYZ } from './learning/airshow';

const params = defaultParams();
const FIXED_DT = 1 / 100; // physics + control rate

const spawn = (): HeliState => {
  const s = initialState(params);
  s.position.z = -15; // start 15 m up (NED: -z is up), room for aerobatics
  return s;
};

/** Soft ground floor at z = 0 (NED): the heli can't sink below the grid. */
const clampGround = (s: HeliState): void => {
  if (s.position.z >= 0) {
    s.position.z = 0;
    const vWorld = rotate(s.orientation, s.velBody);
    if (vWorld.z > 0) {
      vWorld.z = 0; // cancel the downward (into-ground) component
      s.velBody = rotateInverse(s.orientation, vWorld);
    }
  }
};

const canvas = document.getElementById('scene') as HTMLCanvasElement;
const hudEl = document.getElementById('hud') as HTMLElement;

const scene = new HeliScene(canvas);
const input = new InputController(params);
const hud = new Hud(hudEl);
const hover = new HoverController(params, FIXED_DT);

let state = spawn();
let cameraMode = 'chase';
let lastControl: Control = zeroControl();

type Mode = 'manual' | 'hold' | 'maneuver';
let mode: Mode = 'manual';
const MODE_LABEL: Record<Mode, string> = { manual: 'manual', hold: 'hover hold', maneuver: '' };

let traj: TrajectoryController | null = null;
let trajStep = 0;
let activeRef: Reference | null = null;

let gustTimer = 0;
let gustVec: Vec3 = vec3();

// --- Apprenticeship-learning mode. ---
let learningMode = false;
let learn: LearnResult | null = null;
let learnDemoCount = 0;
let snapIdx = 0;
let snapTimer = 0;
let learnSeed = 1;
const SNAP_INTERVAL = 0.7; // seconds per EM iteration shown

const enterOrRegenLearning = (): void => {
  const truth = airshowPath();
  const opts = { ...defaultDemoOptions, seed: learnSeed++ };
  const { demos } = generateDemos(truth, opts);
  learn = learnTrajectory(demos, defaultLearnOptions, truth);
  learnDemoCount = demos.length;
  scene.setLearningCurves(demos.map(toXYZ), toXYZ(truth), toXYZ(learn.snapshots[0]));
  scene.enterLearningView();
  learningMode = true;
  snapIdx = 0;
  snapTimer = 0;
};

const exitLearning = (): void => {
  scene.exitLearningView();
  learningMode = false;
  learn = null;
  toManual();
};

const toManual = (): void => {
  mode = 'manual';
  input.setCollective(lastControl.u4); // resume manual from the current collective
  traj = null;
  activeRef = null;
  scene.hideReference();
  scene.hideSetpoint();
};

const toHold = (): void => {
  mode = 'hold';
  hover.engageFrom(state);
  traj = null;
  activeRef = null;
  scene.hideReference();
  scene.showSetpoint(hover.setpoint.position);
};

const startManeuver = (ref: Reference): void => {
  activeRef = ref;
  traj = new TrajectoryController(params, ref.states, ref.controls, FIXED_DT);
  trajStep = 0;
  mode = 'maneuver';
  scene.hideSetpoint();
  scene.showReference(ref.states.map((s) => s.position));
};

const triggerGust = (): void => {
  const angle = Math.random() * Math.PI * 2;
  const strength = 6 + Math.random() * 3;
  gustVec = vec3(Math.cos(angle) * strength, Math.sin(angle) * strength, (Math.random() - 0.6) * 4);
  gustTimer = 0.18;
};

const trackingError = (): number | undefined => {
  if (mode === 'maneuver' && traj) return Math.hypot(...boxminus(state, traj.referenceState(trajStep)));
  if (mode === 'hold') return Math.hypot(...boxminus(state, hover.setpoint));
  return undefined;
};

let acc = 0;
let prev = performance.now();
let fps = 60;

const frame = (now: number): void => {
  const wall = (now - prev) / 1000;
  prev = now;
  fps = fps * 0.9 + (1 / Math.max(wall, 1e-3)) * 0.1;

  // --- Apprenticeship-learning mode (separate from the flight loop). ---
  if (input.consumeLearning()) enterOrRegenLearning();
  if (learningMode) {
    if (input.consumeManual() || input.consumeReset()) {
      exitLearning();
    } else {
      snapTimer += wall;
      if (learn && snapIdx < learn.snapshots.length - 1 && snapTimer >= SNAP_INTERVAL) {
        snapIdx++;
        snapTimer = 0;
        scene.updateEstimateCurve(toXYZ(learn.snapshots[snapIdx]));
      }
      scene.renderLearning();
      if (learn)
        hud.showLearning(
          {
            iteration: snapIdx,
            totalIterations: learn.snapshots.length - 1,
            rmse: learn.history[snapIdx].rmse!,
            naiveRmse: learn.history[0].rmse!,
            demoCount: learnDemoCount,
            posNoise: defaultDemoOptions.posNoise,
            done: snapIdx >= learn.snapshots.length - 1,
          },
          fps,
        );
      requestAnimationFrame(frame);
      return;
    }
  }

  // --- Discrete events / mode transitions. ---
  if (input.consumeReset()) {
    state = spawn();
    input.resetThrottle();
    scene.resetTrail();
    toManual();
  }
  if (input.consumeCameraCycle()) cameraMode = scene.cycleCamera();
  if (input.consumeGust()) triggerGust();

  const maneuver = input.consumeManeuver();
  if (maneuver) startManeuver(buildManeuver(maneuver, params, state, FIXED_DT));
  if (input.consumeHover()) toHold();
  if (input.consumeManual() || (mode !== 'manual' && input.manualStickActive())) toManual();

  // Manual control is sampled once per frame (the held collective integrates over wall time).
  const manualControl = input.poll(Math.min(wall, 0.05));

  // --- Fixed-step physics + control. ---
  acc += Math.min(wall, 0.1);
  while (acc >= FIXED_DT) {
    const dist = gustTimer > 0 ? { lin: gustVec } : undefined;

    let u: Control;
    if (mode === 'manual') {
      u = manualControl;
    } else if (mode === 'hold') {
      u = hover.control(state);
    } else {
      u = traj!.control(state, trajStep);
      trajStep++;
      if (trajStep >= traj!.length) toHold(); // maneuver complete -> settle into hover
    }

    state = step(params, state, u, FIXED_DT, dist);
    clampGround(state);
    lastControl = u;
    if (gustTimer > 0) gustTimer -= FIXED_DT;
    acc -= FIXED_DT;
  }

  scene.update(state, lastControl, wall);
  hud.update(state, lastControl, cameraMode, fps, now / 1000, {
    mode,
    label: mode === 'maneuver' ? activeRef?.name ?? 'maneuver' : MODE_LABEL[mode],
    trackingError: trackingError(),
  });
  requestAnimationFrame(frame);
};

requestAnimationFrame(frame);
