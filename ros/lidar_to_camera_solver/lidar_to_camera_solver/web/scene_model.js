import * as THREE from "./vendor/three.module.js";

function rmsColor(rms, colored = true) {
  if (!colored || rms == null || !Number.isFinite(Number(rms))) {
    return new THREE.Color(0x8c98a8);
  }
  const value = Math.max(0, Math.min(1, (Number(rms) - 15) / 50));
  return new THREE.Color().setRGB(0.24 + value * 0.70, 0.78 - value * 0.58, 0.35);
}

function pointsGeometry(points) {
  const geometry = new THREE.BufferGeometry();
  const values = [];
  for (const point of points || []) {
    if (Array.isArray(point) && point.length >= 3) values.push(...point.slice(0, 3));
  }
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(values, 3));
  return geometry;
}

function lineGeometry(points) {
  const geometry = new THREE.BufferGeometry();
  const values = [];
  for (const point of points || []) {
    if (Array.isArray(point) && point.length >= 3) values.push(...point.slice(0, 3));
  }
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(values, 3));
  return geometry;
}

function disposeObject(object) {
  object.traverse((child) => {
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach((item) => item.dispose());
      else child.material.dispose();
    }
  });
}

function parseCloud(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 12) return [];
  const view = new DataView(buffer);
  const points = [];
  for (let offset = 0; offset + 12 <= buffer.byteLength; offset += 12) {
    const point = [
      view.getFloat32(offset, true),
      view.getFloat32(offset + 4, true),
      view.getFloat32(offset + 8, true),
    ];
    if (point.every(Number.isFinite)) points.push(point);
  }
  return points;
}

/** Three.js owner for the world-space review scene. */
export class SceneModel {
  constructor(canvas, { onPick = () => {} } = {}) {
    this.canvas = canvas;
    this.onPick = onPick;
    this.captureGroups = new Map();
    this.target = new THREE.Vector3();
    this._cameraSignature = "";
    this._drag = null;
    this.available = true;
    try {
      this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      this.renderer.setClearColor(0x171a1f, 1);
    } catch (error) {
      this.available = false;
      this.error = error instanceof Error ? error.message : String(error);
      canvas.dataset.sceneError = "WebGL unavailable";
      return;
    }

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x171a1f);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
    this.camera.position.set(0, 1.5, 5);
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.cameraGroup = null;
    this.raycaster = new THREE.Raycaster();
    this.raycaster.params.Line.threshold = 0.04;
    this._bindControls();
    this._resize();
    window.addEventListener("resize", () => this._resize());
  }

  _bindControls() {
    this.canvas.addEventListener("contextmenu", (event) => event.preventDefault());
    this.canvas.addEventListener("pointerdown", (event) => {
      if (event.button !== 0 && event.button !== 2) return;
      this.canvas.setPointerCapture(event.pointerId);
      this._drag = {
        button: event.button,
        x: event.clientX,
        y: event.clientY,
        moved: 0,
      };
    });
    this.canvas.addEventListener("pointermove", (event) => {
      if (!this._drag) return;
      const dx = event.clientX - this._drag.x;
      const dy = event.clientY - this._drag.y;
      this._drag.x = event.clientX;
      this._drag.y = event.clientY;
      this._drag.moved += Math.abs(dx) + Math.abs(dy);
      if (this._drag.button === 2) this._orbit(dx, dy);
      else this._pan(dx, dy);
      this._render();
    });
    this.canvas.addEventListener("pointerup", (event) => {
      if (!this._drag) return;
      const drag = this._drag;
      this._drag = null;
      if (drag.button === 0 && drag.moved < 4) this.pick(event.clientX, event.clientY);
    });
    this.canvas.addEventListener("wheel", (event) => {
      event.preventDefault();
      const direction = Math.exp(event.deltaY * 0.0012);
      const offset = this.camera.position.clone().sub(this.target);
      offset.multiplyScalar(Math.max(0.2, Math.min(5, direction)));
      this.camera.position.copy(this.target).add(offset);
      this._render();
    }, { passive: false });
  }

  _resize() {
    if (!this.available) return;
    const width = Math.max(1, this.canvas.clientWidth || this.canvas.width || 1);
    const height = Math.max(1, this.canvas.clientHeight || this.canvas.height || 1);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
    this._render();
  }

  _orbit(dx, dy) {
    const offset = this.camera.position.clone().sub(this.target);
    const spherical = new THREE.Spherical().setFromVector3(offset);
    spherical.theta -= dx * 0.006;
    spherical.phi = Math.max(0.08, Math.min(Math.PI - 0.08, spherical.phi + dy * 0.006));
    this.camera.position.setFromSpherical(spherical).add(this.target);
    this.camera.lookAt(this.target);
  }

  _pan(dx, dy) {
    const distance = this.camera.position.distanceTo(this.target);
    const scale = distance / Math.max(1, this.canvas.clientHeight || 1);
    const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 1);
    const shift = right.multiplyScalar(-dx * scale).add(up.multiplyScalar(dy * scale));
    this.camera.position.add(shift);
    this.target.add(shift);
    this.camera.lookAt(this.target);
  }

  _captureGroup(capture, pair, cloud) {
    const group = new THREE.Group();
    group.userData.captureId = Number(capture.id);
    group.userData.cloudBuffer = cloud;
    const color = rmsColor(pair && pair.rms_px);
    group.userData.color = color;

    const boardMaterial = new THREE.LineBasicMaterial({ color });
    const board = new THREE.LineLoop(lineGeometry(capture.board_outline_world), boardMaterial);
    board.userData.captureId = Number(capture.id);
    group.add(board);
    group.userData.board = board;

    group.userData.markers = [];
    for (const quad of capture.marker_quads_world || []) {
      const marker = new THREE.LineLoop(
        lineGeometry([...(quad || []), (quad || [])[0]]),
        new THREE.LineBasicMaterial({ color: 0xf0a53a }),
      );
      marker.userData.captureId = Number(capture.id);
      group.add(marker);
      group.userData.markers.push(marker);
    }
    this._setCloud(group, cloud);
    group.scale.setScalar(group.userData.captureId === this.selectedId ? 1.03 : 1);
    return group;
  }

  _setCloud(group, cloud) {
    const existing = group.userData.points;
    if (existing) {
      group.remove(existing);
      disposeObject(existing);
      delete group.userData.points;
    }
    if (!(cloud instanceof ArrayBuffer)) return;
    const points = new THREE.Points(
      pointsGeometry(parseCloud(cloud)),
      new THREE.PointsMaterial({ color: group.userData.color, size: 0.018 }),
    );
    points.userData.captureId = group.userData.captureId;
    group.userData.points = points;
    group.add(points);
  }

  _updateGroup(group, pair, cloud, showPoints, colorByRms) {
    const color = rmsColor(pair && pair.rms_px, colorByRms);
    group.userData.color = color;
    group.userData.board.material.color.copy(color);
    for (const marker of group.userData.markers) marker.material.color.set(0xf0a53a);
    if (group.userData.points) {
      group.userData.points.visible = showPoints;
      group.userData.points.material.color.copy(color);
    }
    if (cloud !== group.userData.cloudBuffer) {
      group.userData.cloudBuffer = cloud;
      this._setCloud(group, cloud);
      if (group.userData.points) group.userData.points.visible = showPoints;
    }
    group.scale.setScalar(group.userData.captureId === this.selectedId ? 1.03 : 1);
  }

  _cameraObject(cameraData, showFrustum = true) {
    if (!cameraData || !cameraData.optical_pose_world) return null;
    const pose = cameraData.optical_pose_world;
    const group = new THREE.Group();
    group.position.fromArray(pose.position || [0, 0, 0]);
    group.quaternion.fromArray(pose.orientation || [0, 0, 0, 1]);
    const axisLength = 0.35;
    const axisSpecs = [
      [[0, 0, 0], [axisLength, 0, 0], 0xef5f80],
      [[0, 0, 0], [0, axisLength, 0], 0x4bcf7d],
      [[0, 0, 0], [0, 0, axisLength], 0x4d8fe2],
    ];
    for (const [start, end, color] of axisSpecs) {
      group.add(new THREE.Line(lineGeometry([start, end]), new THREE.LineBasicMaterial({ color })));
    }
    if (!showFrustum) return group;
    const { fx, fy, cx, cy } = cameraData.intrinsics || {};
    const { width, height } = cameraData.image_size || {};
    if (![fx, fy, cx, cy, width, height].every(Number.isFinite) || fx <= 0 || fy <= 0) {
      return group;
    }
    const near = 0.22;
    const far = 0.65;
    const corner = (depth, x, y) => [
      (x - cx) * depth / fx,
      -(y - cy) * depth / fy,
      depth,
    ];
    const nearCorners = [corner(near, 0, 0), corner(near, width, 0), corner(near, width, height), corner(near, 0, height)];
    const farCorners = [corner(far, 0, 0), corner(far, width, 0), corner(far, width, height), corner(far, 0, height)];
    const frustumPoints = [];
    for (let index = 0; index < 4; index += 1) {
      frustumPoints.push([0, 0, 0], nearCorners[index]);
      frustumPoints.push(nearCorners[index], nearCorners[(index + 1) % 4]);
      frustumPoints.push(nearCorners[index], farCorners[index]);
      frustumPoints.push(farCorners[index], farCorners[(index + 1) % 4]);
    }
    group.add(new THREE.LineSegments(
      lineGeometry(frustumPoints),
      new THREE.LineBasicMaterial({ color: 0x96a1af, transparent: true, opacity: 0.6 }),
    ));
    return group;
  }

  sync(app) {
    if (!this.available) return;
    const sceneData = app.scene || { captures: [], camera: null };
    const pairs = new Map((app.state && app.state.pairs || []).map((pair) => [Number(pair.id), pair]));
    const clouds = app.clouds || new Map();
    const seen = new Set();
    for (const capture of sceneData.captures || []) {
      const id = Number(capture.id);
      seen.add(id);
      const pair = pairs.get(id);
      const cloud = clouds.get(id);
      let group = this.captureGroups.get(id);
      if (!group) {
        group = this._captureGroup(capture, pair, cloud);
        this.captureGroups.set(id, group);
        this.root.add(group);
        this._updateGroup(
          group,
          pair,
          cloud,
          app.layers?.points !== false,
          app.layers?.rms !== false,
        );
      } else {
        this._updateGroup(group, pair, cloud, app.layers?.points !== false, app.layers?.rms !== false);
      }
    }
    for (const [id, group] of this.captureGroups) {
      if (seen.has(id)) continue;
      this.root.remove(group);
      disposeObject(group);
      this.captureGroups.delete(id);
    }

    const signature = JSON.stringify([sceneData.camera, app.layers?.frustum !== false]);
    if (signature !== this._cameraSignature) {
      if (this.cameraGroup) {
        this.scene.remove(this.cameraGroup);
        disposeObject(this.cameraGroup);
      }
      this.cameraGroup = this._cameraObject(sceneData.camera, app.layers?.frustum !== false);
      if (this.cameraGroup) this.scene.add(this.cameraGroup);
      this._cameraSignature = signature;
    }
    this._render();
  }

  focus(id) {
    if (!this.available) return;
    const group = this.captureGroups.get(Number(id));
    if (!group) return;
    const box = new THREE.Box3().setFromObject(group);
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.8, 0.8);
    this.selectedId = Number(id);
    this.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(radius, radius * 0.6, radius));
    this.camera.lookAt(this.target);
    for (const item of this.captureGroups.values()) {
      item.scale.setScalar(item.userData.captureId === this.selectedId ? 1.03 : 1);
    }
    this._render();
  }

  frameAll() {
    if (!this.available) return;
    const box = new THREE.Box3().setFromObject(this.root);
    if (this.cameraGroup) box.expandByObject(this.cameraGroup);
    if (box.isEmpty()) {
      this.target.set(0, 0, 0);
      this.camera.position.set(0, 1.5, 5);
    } else {
      const center = box.getCenter(new THREE.Vector3());
      const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.55, 1);
      this.target.copy(center);
      this.camera.position.copy(center).add(new THREE.Vector3(radius, radius * 0.6, radius));
    }
    this.camera.lookAt(this.target);
    this._render();
  }

  pick(clientX, clientY) {
    if (!this.available) return null;
    const rect = this.canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1,
    );
    this.raycaster.setFromCamera(ndc, this.camera);
    const hits = this.raycaster.intersectObjects(this.root.children, true);
    const hit = hits.find((item) => item.object.userData.captureId != null);
    if (!hit) return null;
    const id = Number(hit.object.userData.captureId);
    this.onPick(id);
    return id;
  }

  _render() {
    if (this.available && this.renderer) this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (!this.available) return;
    for (const group of this.captureGroups.values()) disposeObject(group);
    if (this.cameraGroup) disposeObject(this.cameraGroup);
    this.renderer.dispose();
  }
}
