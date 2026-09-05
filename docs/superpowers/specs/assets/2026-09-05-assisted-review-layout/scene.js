/* Shared throwaway mock for the three layout prototypes.
 *
 * This is NOT the proposed renderer. The real one is three.js. This is ~150
 * lines of canvas 2D so the viewport in each prototype is alive rather than a
 * grey box -- you cannot judge a layout against a placeholder rectangle.
 *
 * It fakes the data the real /api/state would carry: four captured pairs, each
 * a board pose plus a small plane-inlier cloud, in the LiDAR frame.
 */

const PAIRS = [
  { id: 0, rms: 18.0, t: [0.20, -0.15, 4.10], rpy: [0.10, 0.20, 0.00] },
  { id: 1, rms: 21.2, t: [-0.90, 0.05, 3.40], rpy: [-0.20, -0.35, 0.10] },
  { id: 2, rms: 36.5, t: [1.10, 0.40, 5.20], rpy: [0.15, 0.50, -0.10] },
  { id: 3, rms: 61.7, t: [-0.30, 0.60, 6.00], rpy: [-0.30, 0.10, 0.25] },
];

const PLATE_HALF = 0.30;   // 600 mm plate
const MARKER_HALF = 0.055;

/* The camera's pose in the LiDAR frame -- in the real app, the solve output.
 * The frustum and the camera's axis marker are both built from THIS, so they
 * share an origin and an orientation by construction. They used to be two
 * unrelated literals, which is why they did not line up. */
const CAMERA = { t: [0.30, 0.02, -0.05], rpy: [0.0, -0.06, 0.0] };

const AXES = [
  { end: [0.4, 0, 0], color: '#e2564d' },
  { end: [0, 0.4, 0], color: '#55c46a' },
  { end: [0, 0, 0.4], color: '#4d8fe2' },
];

function rotation(rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

function toWorld(pair, local) {
  const R = rotation(pair.rpy);
  return [0, 1, 2].map(
    (i) => R[i][0] * local[0] + R[i][1] * local[1] + R[i][2] * local[2] + pair.t[i]
  );
}

/* One place that maps camera-local metres into the LiDAR frame. */
const cameraToWorld = (local) => toWorld(CAMERA, local);
const identity = (p) => p;

/* Board-local geometry: the plate outline, four marker quads, and a scatter of
 * points standing in for debug/plane_inliers. */
function buildGeometry() {
  let seed = 7;
  const rand = () => {
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    return seed / 0x7fffffff;
  };
  return PAIRS.map((pair) => {
    const plate = [
      [-PLATE_HALF, -PLATE_HALF, 0], [PLATE_HALF, -PLATE_HALF, 0],
      [PLATE_HALF, PLATE_HALF, 0], [-PLATE_HALF, PLATE_HALF, 0],
    ].map((c) => toWorld(pair, c));

    const markers = [[-0.15, -0.15], [0.15, -0.15], [0.15, 0.15], [-0.15, 0.15]]
      .map(([mx, my]) => [
        [mx - MARKER_HALF, my - MARKER_HALF, 0.001],
        [mx + MARKER_HALF, my - MARKER_HALF, 0.001],
        [mx + MARKER_HALF, my + MARKER_HALF, 0.001],
        [mx - MARKER_HALF, my + MARKER_HALF, 0.001],
      ].map((c) => toWorld(pair, c)));

    const points = [];
    for (let i = 0; i < 220; i++) {
      points.push(toWorld(pair, [
        (rand() * 2 - 1) * PLATE_HALF,
        (rand() * 2 - 1) * PLATE_HALF,
        (rand() - 0.5) * 0.02,
      ]));
    }
    return { pair, plate, markers, points, centre: pair.t };
  });
}

const GEOMETRY = buildGeometry();

/* Colour a pair by reprojection RMS: good is green, bad is red. */
function rmsColor(rms, alpha) {
  const k = Math.max(0, Math.min(1, (rms - 15) / 50));
  const r = Math.round(60 + k * 195);
  const g = Math.round(200 - k * 150);
  return `rgba(${r},${g},90,${alpha})`;
}

class Viewport {
  constructor(canvas, options = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.yaw = -2.5;
    this.pitch = 0.35;
    this.dist = 9;
    this.target = [0, 0, 4.5];
    this.selected = null;
    this.showPoints = true;
    this.showFrustum = true;
    this.colorByRms = true;
    this.pointSize = 2;
    this.theme = options.theme || 'dark';
    this.onPick = options.onPick || (() => {});
    this._bind();
    this._resize();
    window.addEventListener('resize', () => { this._resize(); this.draw(); });
  }

  _bind() {
    // Left drags the scene flat (pan), right swings the camera round it (orbit).
    // A left press that never moves is a pick, so selecting a pair costs one
    // click and does not fight the pan.
    let mode = null, lastX = 0, lastY = 0, moved = 0;
    this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    this.canvas.addEventListener('mousedown', (e) => {
      if (e.button === 0) mode = 'pan';
      else if (e.button === 2) mode = 'orbit';
      else return;
      e.preventDefault();
      lastX = e.clientX; lastY = e.clientY; moved = 0;
      this.canvas.style.cursor = mode === 'pan' ? 'grabbing' : 'move';
    });
    window.addEventListener('mouseup', (e) => {
      if (!mode) return;
      if (mode === 'pan' && e.button === 0 && moved < 4) this._pick(e);
      mode = null;
      this.canvas.style.cursor = 'grab';
    });
    window.addEventListener('mousemove', (e) => {
      if (!mode) return;
      const dx = e.clientX - lastX, dy = e.clientY - lastY;
      moved += Math.abs(dx) + Math.abs(dy);
      lastX = e.clientX; lastY = e.clientY;
      if (mode === 'orbit') {
        // Horizontal is inverted relative to the naive mapping: dragging right
        // swings the scene right, so the camera goes left. Grabbing the world,
        // not the camera -- which is what every 3D tool does.
        this.yaw += dx * 0.006;
        this.pitch = Math.max(-1.4, Math.min(1.4, this.pitch + dy * 0.006));
      } else {
        // Scaled by distance and by the same focal the projection uses, so a
        // dragged point tracks the cursor 1:1 at any zoom.
        const { right, up } = this._basis();
        const k = this.dist / (this.h * 0.9);
        for (let i = 0; i < 3; i++) {
          this.target[i] += (up[i] * dy - right[i] * dx) * k;
        }
      }
      this.draw();
    });
    this.canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.dist = Math.max(1.2, Math.min(24, this.dist * (1 + e.deltaY * 0.0012)));
      this.draw();
    }, { passive: false });
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, rect.width * dpr);
    this.canvas.height = Math.max(1, rect.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.w = rect.width; this.h = rect.height;
  }

  _basis() {
    const cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
    const eye = [
      this.target[0] + this.dist * cp * Math.cos(this.yaw),
      this.target[1] + this.dist * sp,
      this.target[2] + this.dist * cp * Math.sin(this.yaw),
    ];
    const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    const norm = (v) => { const n = Math.hypot(...v); return v.map((x) => x / n); };
    const cross = (a, b) => [
      a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0],
    ];
    const fwd = norm(sub(this.target, eye));
    const right = norm(cross(fwd, [0, 1, 0]));
    const up = cross(right, fwd);
    return { eye, fwd, right, up };
  }

  project(p) {
    const { eye, fwd, right, up } = this._basis();
    const d = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
    const z = d[0] * fwd[0] + d[1] * fwd[1] + d[2] * fwd[2];
    if (z < 0.05) return null;
    const x = d[0] * right[0] + d[1] * right[1] + d[2] * right[2];
    const y = d[0] * up[0] + d[1] * up[1] + d[2] * up[2];
    const f = this.h * 0.9;
    return { x: this.w / 2 + (x * f) / z, y: this.h / 2 - (y * f) / z, z };
  }

  focus(id) {
    const g = GEOMETRY.find((g) => g.pair.id === id);
    if (!g) return;
    this.selected = id;
    this.target = g.centre.slice();
    this.dist = 2.6;
    this.draw();
  }

  frameAll() {
    this.selected = null;
    this.target = [0, 0, 4.5];
    this.dist = 9;
    this.draw();
  }

  _pick(event) {
    const rect = this.canvas.getBoundingClientRect();
    const mx = event.clientX - rect.left, my = event.clientY - rect.top;
    let best = null, bestD = 46;
    for (const g of GEOMETRY) {
      const p = this.project(g.centre);
      if (!p) continue;
      const d = Math.hypot(p.x - mx, p.y - my);
      if (d < bestD) { bestD = d; best = g.pair.id; }
    }
    if (best !== null) this.onPick(best);
  }

  /* One axis triad. `toWorld` maps that frame's local metres into the LiDAR
   * frame, so the origin dot, the three axes and any geometry built from the
   * same map cannot drift apart. */
  _drawAxes(toWorld, label, dark) {
    const c = this.ctx;
    const o = this.project(toWorld([0, 0, 0]));
    if (!o) return;
    c.lineWidth = 1.6;
    for (const axis of AXES) {
      const e = this.project(toWorld(axis.end));
      if (!e) continue;
      c.strokeStyle = axis.color;
      c.beginPath(); c.moveTo(o.x, o.y); c.lineTo(e.x, e.y); c.stroke();
    }
    c.fillStyle = dark ? '#e2e7ee' : '#1c2430';
    c.beginPath(); c.arc(o.x, o.y, 2.5, 0, Math.PI * 2); c.fill();
    c.font = '11px ui-monospace, monospace';
    c.fillStyle = dark ? 'rgba(226,231,238,.7)' : 'rgba(28,36,48,.7)';
    c.fillText(label, o.x + 7, o.y - 7);
  }

  draw() {
    const c = this.ctx, dark = this.theme === 'dark';
    c.clearRect(0, 0, this.w, this.h);
    c.fillStyle = dark ? '#0b0d10' : '#eef1f5';
    c.fillRect(0, 0, this.w, this.h);

    // ground grid, so orbiting reads as 3D
    c.strokeStyle = dark ? 'rgba(255,255,255,.07)' : 'rgba(0,0,0,.09)';
    c.lineWidth = 1;
    for (let i = -6; i <= 6; i++) {
      for (const seg of [
        [[i, -1.4, -2], [i, -1.4, 10]],
        [[-6, -1.4, i + 4], [6, -1.4, i + 4]],
      ]) {
        const a = this.project(seg[0]), b = this.project(seg[1]);
        if (a && b) { c.beginPath(); c.moveTo(a.x, a.y); c.lineTo(b.x, b.y); c.stroke(); }
      }
    }

    // Both sensor frames, each drawn through its own local-to-world map.
    this._drawAxes(identity, 'velodyne', dark);

    // Camera frustum, drawn only where a solve exists. Apex and axis marker
    // come from the same pose, so the cone converges exactly on the triad.
    if (this.showFrustum) {
      const apex = this.project(cameraToWorld([0, 0, 0]));
      const corners = [
        [-0.9, -0.7, 1.6], [0.9, -0.7, 1.6], [0.9, 0.7, 1.6], [-0.9, 0.7, 1.6],
      ].map((p) => this.project(cameraToWorld(p)));
      if (apex && corners.every(Boolean)) {
        c.strokeStyle = dark ? 'rgba(120,200,255,.5)' : 'rgba(0,90,190,.5)';
        c.lineWidth = 1;
        for (const q of corners) {
          c.beginPath(); c.moveTo(apex.x, apex.y); c.lineTo(q.x, q.y); c.stroke();
        }
        c.beginPath();
        corners.forEach((q, i) => (i ? c.lineTo(q.x, q.y) : c.moveTo(q.x, q.y)));
        c.closePath(); c.stroke();
      }
      this._drawAxes(cameraToWorld, 'camera', dark);
    }

    // every pair, painter's algorithm
    const polys = [];
    for (const g of GEOMETRY) {
      const dim = this.selected !== null && this.selected !== g.pair.id;
      const base = this.colorByRms
        ? rmsColor(g.pair.rms, dim ? 0.14 : 0.42)
        : `rgba(120,170,255,${dim ? 0.14 : 0.42})`;
      const pts = g.plate.map((p) => this.project(p));
      if (pts.every(Boolean)) {
        polys.push({ pts, fill: base, stroke: this.colorByRms
          ? rmsColor(g.pair.rms, dim ? 0.3 : 1)
          : `rgba(150,200,255,${dim ? 0.3 : 1})`,
          z: pts.reduce((s, p) => s + p.z, 0) / 4, width: this.selected === g.pair.id ? 2.5 : 1.2 });
      }
      for (const m of g.markers) {
        const mp = m.map((p) => this.project(p));
        if (mp.every(Boolean)) {
          polys.push({ pts: mp, fill: `rgba(20,20,25,${dim ? 0.25 : 0.85})`,
            stroke: `rgba(255,255,255,${dim ? 0.12 : 0.5})`,
            z: mp.reduce((s, p) => s + p.z, 0) / 4 - 0.001, width: 1 });
        }
      }
      if (this.showPoints) {
        for (const p of g.points) {
          const q = this.project(p);
          if (!q) continue;
          polys.push({ point: q, fill: this.colorByRms
            ? rmsColor(g.pair.rms, dim ? 0.2 : 0.9)
            : `rgba(200,225,255,${dim ? 0.2 : 0.9})`, z: q.z });
        }
      }
    }
    polys.sort((a, b) => b.z - a.z);
    for (const poly of polys) {
      if (poly.point) {
        c.fillStyle = poly.fill;
        c.fillRect(poly.point.x, poly.point.y, this.pointSize, this.pointSize);
        continue;
      }
      c.beginPath();
      poly.pts.forEach((p, i) => (i ? c.lineTo(p.x, p.y) : c.moveTo(p.x, p.y)));
      c.closePath();
      c.fillStyle = poly.fill; c.fill();
      c.strokeStyle = poly.stroke; c.lineWidth = poly.width; c.stroke();
    }

    // labels last, so they never sit under a plate
    c.font = '11px ui-monospace, monospace';
    for (const g of GEOMETRY) {
      const p = this.project(g.centre);
      if (!p) continue;
      const on = this.selected === null || this.selected === g.pair.id;
      c.fillStyle = dark
        ? `rgba(255,255,255,${on ? 0.9 : 0.25})`
        : `rgba(20,25,35,${on ? 0.9 : 0.25})`;
      c.fillText(`#${g.pair.id}  ${g.pair.rms.toFixed(1)} px`, p.x + 8, p.y);
    }
  }
}

/* A stand-in for /api/pair/<id>/preview.jpg: a board with corner marks drawn on. */
function drawArucoPreview(canvas, id) {
  const c = canvas.getContext('2d');
  const w = canvas.width, h = canvas.height;
  c.fillStyle = '#2a2e33'; c.fillRect(0, 0, w, h);
  const g = c.createLinearGradient(0, 0, w, h);
  g.addColorStop(0, '#4a4f57'); g.addColorStop(1, '#31353b');
  c.fillStyle = g; c.fillRect(0, 0, w, h);

  const cx = w * (0.36 + 0.09 * id), cy = h * (0.55 - 0.05 * id), s = w * 0.24;
  c.save();
  c.translate(cx, cy);
  c.rotate((id - 1.5) * 0.09);
  c.fillStyle = '#d8d4cc';
  c.fillRect(-s, -s, s * 2, s * 2);
  for (const [mx, my] of [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]) {
    c.fillStyle = '#15171a';
    c.fillRect(mx * s - s * 0.19, my * s - s * 0.19, s * 0.38, s * 0.38);
    c.fillStyle = '#d8d4cc';
    c.fillRect(mx * s - s * 0.1, my * s - s * 0.1, s * 0.09, s * 0.09);
    c.fillRect(mx * s + s * 0.02, my * s + s * 0.02, s * 0.09, s * 0.09);
    // detected corners, at the thickness preview.py now uses
    c.strokeStyle = '#00ff00'; c.lineWidth = 3;
    c.strokeRect(mx * s - s * 0.19, my * s - s * 0.19, s * 0.38, s * 0.38);
    c.strokeStyle = '#ff8000'; c.lineWidth = 2;
    const rx = mx * s - s * 0.19 + 4, ry = my * s - s * 0.19 + 3;
    c.beginPath(); c.moveTo(rx - 7, ry); c.lineTo(rx + 7, ry);
    c.moveTo(rx, ry - 7); c.lineTo(rx, ry + 7); c.stroke();
  }
  c.restore();
  c.fillStyle = 'rgba(0,0,0,.55)';
  c.fillRect(0, h - 20, w, 20);
  c.fillStyle = '#cfd6df';
  c.font = '11px ui-monospace, monospace';
  c.fillText(`pair #${id}   green = detected   orange = reprojected`, 8, h - 6);
}
