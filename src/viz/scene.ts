import * as THREE from 'three';
import { HeliState, Control } from '../physics/heli';
import { toThreePosition, toThreeQuaternion } from './coords';

export type CameraMode = 'chase' | 'orbit' | 'top';

const RED = 0xd6353b;
const DARK = 0x222428;
const ACCENT = 0xe8e8ea;

/**
 * Stylized, diagrammatic helicopter visualization. The mesh is modeled in the
 * body NED frame (x forward, y right, z down); the root group's transform maps
 * it into Three's y-up world each frame, so geometry reads the same as the math.
 */
export class HeliScene {
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  private readonly renderer: THREE.WebGLRenderer;

  private readonly heli = new THREE.Group();
  private readonly mainRotor = new THREE.Group();
  private readonly tailRotor = new THREE.Group();
  private thrustArrow!: THREE.ArrowHelper;

  private readonly trail: THREE.Vector3[] = [];
  private readonly trailLine: THREE.Line;
  private static readonly TRAIL_MAX = 600;

  private readonly refLine: THREE.Line;
  private readonly setpointMarker: THREE.Mesh;

  private cameraMode: CameraMode = 'chase';
  private orbitAngle = 0;

  private readonly learningGroup = new THREE.Group();
  private estimateLine: THREE.Line | null = null;
  private learningActive = false;
  private readonly learningCenter = new THREE.Vector3();
  private learningRadius = 24;

  constructor(canvas: HTMLCanvasElement) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

    this.scene.fog = new THREE.Fog(0x1a2336, 50, 160);

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
    this.camera.position.set(6, 4, 8);

    this.addSky();
    this.addLights();
    this.addGround();
    this.buildHelicopter();

    // Recent-path trail.
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(HeliScene.TRAIL_MAX * 3), 3));
    geo.setDrawRange(0, 0);
    this.trailLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0x4fd1c5, transparent: true, opacity: 0.7 }));
    this.scene.add(this.trailLine);

    // Reference (target) trajectory ghost, shown during autopilot maneuvers.
    this.refLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: 0xf6b73c, dashSize: 0.4, gapSize: 0.25, transparent: true, opacity: 0.85 }));
    this.refLine.visible = false;
    this.scene.add(this.refLine);

    // Setpoint marker, shown during hover-hold.
    this.setpointMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xf6b73c, transparent: true, opacity: 0.5, wireframe: true }),
    );
    this.setpointMarker.visible = false;
    this.scene.add(this.setpointMarker);

    this.learningGroup.visible = false;
    this.scene.add(this.learningGroup);

    this.resize();
    window.addEventListener('resize', () => this.resize());
  }

  /** Vertical gradient sky (bright up, dark down) so "which way is up" reads instantly. */
  private addSky(): void {
    const sky = new THREE.Mesh(
      new THREE.SphereGeometry(400, 32, 16),
      new THREE.ShaderMaterial({
        side: THREE.BackSide,
        depthWrite: false,
        uniforms: {
          topColor: { value: new THREE.Color(0x4f87c4) },
          horizonColor: { value: new THREE.Color(0x9fb4cc) },
          bottomColor: { value: new THREE.Color(0x0a0e14) },
        },
        vertexShader: `
          varying vec3 vDir;
          void main() {
            vDir = normalize(position);
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
          }`,
        fragmentShader: `
          varying vec3 vDir;
          uniform vec3 topColor; uniform vec3 horizonColor; uniform vec3 bottomColor;
          void main() {
            float h = vDir.y;
            vec3 c = h > 0.0
              ? mix(horizonColor, topColor, smoothstep(0.0, 0.5, h))
              : mix(horizonColor, bottomColor, smoothstep(0.0, -0.3, h));
            gl_FragColor = vec4(c, 1.0);
          }`,
      }),
    );
    sky.renderOrder = -1;
    this.scene.add(sky);
  }

  private addLights(): void {
    this.scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x202024, 0.9));
    const sun = new THREE.DirectionalLight(0xffffff, 1.1);
    sun.position.set(10, 20, 8);
    this.scene.add(sun);
  }

  private addGround(): void {
    // Solid floor just under the grid, so everything below the grid reads as ground.
    const ground = new THREE.Mesh(
      new THREE.CircleGeometry(300, 64),
      new THREE.MeshStandardMaterial({ color: 0x141b24, roughness: 1, metalness: 0 }),
    );
    ground.rotation.x = -Math.PI / 2; // lie flat, facing up
    ground.position.y = -0.02;
    this.scene.add(ground);

    const grid = new THREE.GridHelper(200, 100, 0x4a5568, 0x2a3340);
    this.scene.add(grid);
    // World frame axes at the origin (Three coords): a teaching aid.
    const worldAxes = new THREE.AxesHelper(2);
    this.scene.add(worldAxes);
  }

  private buildHelicopter(): void {
    const bodyMat = new THREE.MeshStandardMaterial({ color: RED, metalness: 0.2, roughness: 0.5 });
    const darkMat = new THREE.MeshStandardMaterial({ color: DARK, metalness: 0.3, roughness: 0.6 });
    const accentMat = new THREE.MeshStandardMaterial({ color: ACCENT, metalness: 0.1, roughness: 0.8 });

    // Fuselage: elongated along +x (forward). Body up is -z, so the canopy sits at -z.
    const fuselage = new THREE.Mesh(new THREE.SphereGeometry(0.35, 24, 16), bodyMat);
    fuselage.scale.set(1.6, 1.0, 0.9);
    this.heli.add(fuselage);

    const canopy = new THREE.Mesh(new THREE.SphereGeometry(0.22, 20, 14), new THREE.MeshStandardMaterial({ color: 0x2a3340, metalness: 0.1, roughness: 0.2 }));
    canopy.position.set(0.28, 0, -0.08);
    this.heli.add(canopy);

    // Tail boom along -x, with a fin.
    const boom = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.06, 0.06), darkMat);
    boom.position.set(-0.7, 0, -0.02);
    this.heli.add(boom);
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.12, 0.04, 0.22), bodyMat);
    fin.position.set(-1.15, 0, -0.12);
    this.heli.add(fin);

    // Skids below (+z).
    for (const side of [-0.22, 0.22]) {
      const skid = new THREE.Mesh(new THREE.BoxGeometry(0.9, 0.03, 0.03), accentMat);
      skid.position.set(0, side, 0.28);
      this.heli.add(skid);
    }

    // Main rotor: disk in the body x-y plane (normal = body z). The sub-group
    // spins about its local z (the rotor shaft).
    const mainMast = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.18, 8), darkMat);
    mainMast.position.set(0, 0, -0.16);
    this.heli.add(mainMast);
    this.mainRotor.position.set(0, 0, -0.26);
    const mainDisk = new THREE.Mesh(new THREE.CylinderGeometry(0.72, 0.72, 0.01, 32), new THREE.MeshStandardMaterial({ color: 0x88909c, transparent: true, opacity: 0.18, side: THREE.DoubleSide }));
    mainDisk.rotation.x = Math.PI / 2; // cylinder axis y -> z (body shaft)
    this.mainRotor.add(mainDisk);
    for (const ang of [0, Math.PI / 2, Math.PI]) {
      const blade = new THREE.Mesh(new THREE.BoxGeometry(1.44, 0.06, 0.012), darkMat);
      blade.rotation.z = ang;
      this.mainRotor.add(blade);
    }
    this.heli.add(this.mainRotor);

    // Tail rotor: disk in the body x-z plane (normal = body y); spins about local y.
    this.tailRotor.position.set(-1.18, 0.06, -0.12);
    const tailDisk = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.01, 20), new THREE.MeshStandardMaterial({ color: 0x88909c, transparent: true, opacity: 0.2, side: THREE.DoubleSide }));
    this.tailRotor.add(tailDisk); // cylinder axis already y (body lateral)
    for (const ang of [0, Math.PI / 2]) {
      const blade = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.06, 0.36), darkMat);
      blade.rotation.y = ang;
      this.tailRotor.add(blade);
    }
    this.heli.add(this.tailRotor);

    // Body-frame axes (x red / y green / z blue) — teaching aid.
    this.heli.add(new THREE.AxesHelper(0.9));

    // Thrust vector: points along body -z (up) from the hub; length ∝ collective.
    this.thrustArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0, -0.26), 1.2, 0x4fd1c5, 0.25, 0.14);
    this.heli.add(this.thrustArrow);

    this.heli.matrixAutoUpdate = true;
    this.scene.add(this.heli);
  }

  setCameraMode(mode: CameraMode): void {
    this.cameraMode = mode;
  }

  cycleCamera(): CameraMode {
    const order: CameraMode[] = ['chase', 'orbit', 'top'];
    this.cameraMode = order[(order.indexOf(this.cameraMode) + 1) % order.length];
    return this.cameraMode;
  }

  /** Sync the visualization to a physics state + control, and render. */
  update(state: HeliState, control: Control, dt: number): void {
    toThreePosition(state.position, this.heli.position);
    toThreeQuaternion(state.orientation, this.heli.quaternion);

    // Spin rotors (visual rate scaled down from the true rad/s).
    this.mainRotor.rotation.z += state.rotorSpeed * dt * 0.25;
    this.tailRotor.rotation.y += state.rotorSpeed * dt * 0.5;

    // Thrust arrow length tracks collective magnitude.
    const thrust = Math.max(0.05, Math.abs(control.u4)) * 2.2;
    this.thrustArrow.setLength(thrust, 0.22, 0.12);

    this.pushTrail(this.heli.position);
    this.updateCamera();
    this.renderer.render(this.scene, this.camera);
  }

  private pushTrail(p: THREE.Vector3): void {
    const last = this.trail[this.trail.length - 1];
    if (last && last.distanceToSquared(p) < 0.01) return;
    this.trail.push(p.clone());
    if (this.trail.length > HeliScene.TRAIL_MAX) this.trail.shift();
    const attr = this.trailLine.geometry.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < this.trail.length; i++) {
      const v = this.trail[i];
      attr.setXYZ(i, v.x, v.y, v.z);
    }
    attr.needsUpdate = true;
    this.trailLine.geometry.setDrawRange(0, this.trail.length);
  }

  resetTrail(): void {
    this.trail.length = 0;
    this.trailLine.geometry.setDrawRange(0, 0);
  }

  /** Draw a reference trajectory (NED positions) as a dashed ghost line. */
  showReference(positions: { x: number; y: number; z: number }[]): void {
    const pts = positions.map((p) => toThreePosition(p));
    this.refLine.geometry.dispose();
    this.refLine.geometry = new THREE.BufferGeometry().setFromPoints(pts);
    (this.refLine as THREE.Line).computeLineDistances();
    this.refLine.visible = true;
  }

  hideReference(): void {
    this.refLine.visible = false;
  }

  /** Show/hide the hover setpoint marker at a NED position. */
  showSetpoint(p: { x: number; y: number; z: number }): void {
    toThreePosition(p, this.setpointMarker.position);
    this.setpointMarker.visible = true;
  }

  hideSetpoint(): void {
    this.setpointMarker.visible = false;
  }

  // --- Apprenticeship-learning visualization -------------------------------

  private polyline(
    positionsNED: { x: number; y: number; z: number }[],
    color: number,
    opacity: number,
  ): THREE.Line {
    const pts = positionsNED.map((p) => toThreePosition(p));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
  }

  /** Switch into the learning view: hide the helicopter, show the curve group. */
  enterLearningView(): void {
    this.heli.visible = false;
    this.trailLine.visible = false;
    this.refLine.visible = false;
    this.setpointMarker.visible = false;
    this.learningGroup.visible = true;
    this.learningActive = true;
  }

  exitLearningView(): void {
    this.clearLearning();
    this.learningGroup.visible = false;
    this.learningActive = false;
    this.heli.visible = true;
  }

  private clearLearning(): void {
    for (const child of [...this.learningGroup.children]) {
      this.learningGroup.remove(child);
      ((child as THREE.Line).geometry as THREE.BufferGeometry).dispose();
    }
    this.estimateLine = null;
  }

  /** Draw the demonstrations (faint), the ground truth (green) and the current estimate (amber). */
  setLearningCurves(demos: { x: number; y: number; z: number }[][], truth: { x: number; y: number; z: number }[], estimate: { x: number; y: number; z: number }[]): void {
    this.clearLearning();
    for (const d of demos) this.learningGroup.add(this.polyline(d, 0x8b97a8, 0.14));
    this.learningGroup.add(this.polyline(truth, 0x5ee0a0, 0.95));
    this.estimateLine = this.polyline(estimate, 0xf6b73c, 1);
    this.learningGroup.add(this.estimateLine);

    // Orbit center = truth centroid (Three coords); radius from its extent.
    const pts = truth.map((p) => toThreePosition(p));
    this.learningCenter.set(0, 0, 0);
    for (const p of pts) this.learningCenter.add(p);
    this.learningCenter.multiplyScalar(1 / pts.length);
    let r = 0;
    for (const p of pts) r = Math.max(r, p.distanceTo(this.learningCenter));
    this.learningRadius = r * 2.1 + 6;
  }

  updateEstimateCurve(estimate: { x: number; y: number; z: number }[]): void {
    if (!this.estimateLine) return;
    this.estimateLine.geometry.dispose();
    this.estimateLine.geometry = new THREE.BufferGeometry().setFromPoints(estimate.map((p) => toThreePosition(p)));
  }

  /** Render one frame of the learning view (orbits the trajectory). */
  renderLearning(): void {
    this.updateCamera();
  }

  private updateCamera(): void {
    if (this.learningActive) {
      this.orbitAngle += 0.0035;
      this.camera.position.set(
        this.learningCenter.x + Math.cos(this.orbitAngle) * this.learningRadius,
        this.learningCenter.y + this.learningRadius * 0.5,
        this.learningCenter.z + Math.sin(this.orbitAngle) * this.learningRadius,
      );
      this.camera.lookAt(this.learningCenter);
      this.renderer.render(this.scene, this.camera);
      return;
    }
    const target = this.heli.position;
    if (this.cameraMode === 'chase') {
      // Sit behind/above the helicopter's heading.
      const back = new THREE.Vector3(0, 0, 1).applyQuaternion(this.heli.quaternion).setY(0).normalize();
      const desired = target.clone().add(back.multiplyScalar(8)).add(new THREE.Vector3(0, 3.5, 0));
      this.camera.position.lerp(desired, 0.08);
    } else if (this.cameraMode === 'orbit') {
      this.orbitAngle += 0.004;
      this.camera.position.set(target.x + Math.cos(this.orbitAngle) * 10, target.y + 5, target.z + Math.sin(this.orbitAngle) * 10);
    } else {
      this.camera.position.set(target.x + 0.001, target.y + 16, target.z);
    }
    this.camera.lookAt(target);
  }

  resize(): void {
    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth || window.innerWidth;
    const h = canvas.clientHeight || window.innerHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}
