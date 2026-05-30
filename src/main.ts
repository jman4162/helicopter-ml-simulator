import { defaultParams, initialState, step, HeliState } from './physics/heli';
import { HeliScene } from './viz/scene';
import { InputController } from './ui/input';
import { Hud } from './ui/hud';

const params = defaultParams();

const spawn = (): HeliState => {
  const s = initialState(params);
  s.position.z = -6; // start 6 m up (NED: -z is up)
  return s;
};

const canvas = document.getElementById('scene') as HTMLCanvasElement;
const hudEl = document.getElementById('hud') as HTMLElement;

const scene = new HeliScene(canvas);
const input = new InputController(params);
const hud = new Hud(hudEl);

let state = spawn();
let cameraMode = 'chase';

// Fixed-timestep physics with an accumulator; render once per animation frame.
const FIXED_DT = 1 / 200;
let acc = 0;
let prev = performance.now();
let fps = 60;

const frame = (now: number): void => {
  const wall = (now - prev) / 1000;
  prev = now;
  fps = fps * 0.9 + (1 / Math.max(wall, 1e-3)) * 0.1;

  if (input.consumeReset()) {
    state = spawn();
    input.resetThrottle();
    scene.resetTrail();
  }
  if (input.consumeCameraCycle()) cameraMode = scene.cycleCamera();

  const control = input.poll(Math.min(wall, 0.05));

  acc += Math.min(wall, 0.1); // clamp to avoid spiral-of-death after a stall
  while (acc >= FIXED_DT) {
    state = step(params, state, control, FIXED_DT);
    acc -= FIXED_DT;
  }

  scene.update(state, control, wall);
  hud.update(state, control, cameraMode, fps, now / 1000);
  requestAnimationFrame(frame);
};

requestAnimationFrame(frame);
