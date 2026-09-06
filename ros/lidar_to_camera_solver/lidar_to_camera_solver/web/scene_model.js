import * as THREE from "./vendor/three.module.js";
import { rmsColorHex } from "./quality.js";

function rmsColor(rms, colored = true) {
  return new THREE.Color(rmsColorHex(rms, colored));
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
      const materials = Array.isArray(child.material) ? child.material : [child.material];
      for (const material of materials) {
        if (material.map) material.map.dispose();
        if (material.alphaMap) material.alphaMap.dispose();
        material.dispose();
      }
    }
  });
}

function textSprite(text, color = "#e2e7ee", height = 0.12) {
  if (typeof document === "undefined") return null;
  const canvas = document.createElement("canvas");
  canvas.height = 64;
  const context = canvas.getContext("2d");
  if (!context) return null;
  context.font = "600 28px ui-sans-serif, system-ui, sans-serif";
  canvas.width = Math.ceil(context.measureText(String(text)).width + 16);
  context.font = "600 28px ui-sans-serif, system-ui, sans-serif";
  context.clearRect(0, 0, canvas.width, canvas.height);
  context.fillStyle = color;
  context.textBaseline = "middle";
  context.fillText(String(text), 8, canvas.height / 2);
  const texture = new THREE.CanvasTexture(canvas);
  texture.needsUpdate = true;
  const material = new THREE.SpriteMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    depthWrite: false,
  });
  const sprite = new THREE.Sprite(material);
  sprite.scale.set((canvas.width / canvas.height) * height, height, 1);
  return sprite;
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
    // LCTK world frames follow Autoware: x front, y left, z up.
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(4, 2.5, 3);
    this.root = new THREE.Group();
    this.scene.add(this.root);
    this.referenceGroup = null;
    this._referenceSignature = "";
    this.cameraGroup = null;
    this.raycaster = new THREE.Raycaster();
    this.raycaster.params.Line.threshold = 0.04;
    this._bindControls();
    this.camera.lookAt(this.target);
    this._setOrbitFromCamera();
    this._resize();
    this._onWindowResize = () => this._resize();
    window.addEventListener("resize", this._onWindowResize);
    this._resizeObserver = null;
    if (typeof ResizeObserver === "function") {
      this._resizeObserver = new ResizeObserver(() => this._resize());
      this._resizeObserver.observe(canvas.parentElement || canvas);
    }
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
      this._setOrbitFromCamera();
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

  _setOrbitFromCamera() {
    const offset = this.camera.position.clone().sub(this.target);
    this._orbitRadius = Math.max(offset.length(), 0.001);
    this._orbitYaw = Math.atan2(offset.y, offset.x);
    this._orbitPitch = Math.atan2(offset.z, Math.hypot(offset.x, offset.y));
  }

  _setCameraFromOrbit() {
    const horizontal = Math.cos(this._orbitPitch) * this._orbitRadius;
    this.camera.position.set(
      this.target.x + horizontal * Math.cos(this._orbitYaw),
      this.target.y + horizontal * Math.sin(this._orbitYaw),
      this.target.z + Math.sin(this._orbitPitch) * this._orbitRadius,
    );
    this.camera.lookAt(this.target);
  }

  _orbit(dx, dy) {
    // Keep yaw/pitch unbounded: crossing either pole continues the orbit
    // instead of stopping against an artificial vertical wall.  Vertical
    // drag is inverted to match the operator's view convention.
    this._orbitYaw -= dx * 0.006;
    this._orbitPitch -= dy * 0.006;
    this._setCameraFromOrbit();
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
    const cameraLabel = textSprite("camera optical", "#e2e7ee");
    if (cameraLabel) {
      cameraLabel.position.set(0, 0.08, 0);
      group.add(cameraLabel);
    }
    const axisLabels = [
      ["X", [axisLength * 1.15, 0, 0], "#ef5f80"],
      ["Y", [0, axisLength * 1.15, 0], "#4bcf7d"],
      ["Z", [0, 0, axisLength * 1.15], "#4d8fe2"],
    ];
    for (const [label, position, color] of axisLabels) {
      const sprite = textSprite(label, color);
      if (!sprite) continue;
      sprite.position.fromArray(position);
      group.add(sprite);
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

  _referenceObject(frameId) {
    const group = new THREE.Group();
    const frame = String(frameId || "world");
    const grid = new THREE.GridHelper(8, 16, 0x44515f, 0x2d363f);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = 0;
    if (grid.material) {
      const materials = Array.isArray(grid.material) ? grid.material : [grid.material];
      for (const material of materials) {
        material.transparent = true;
        material.opacity = 0.55;
        material.depthWrite = false;
      }
    }
    group.add(grid);

    const axes = new THREE.AxesHelper(0.45);
    group.add(axes);
    const originLabel = textSprite(`LiDAR (${frame})`, "#f0a53a");
    if (originLabel) {
      originLabel.position.set(0.05, 0.08, 0.04);
      group.add(originLabel);
    }
    const gridLabel = textSprite("LiDAR XY reference", "#96a1af");
    if (gridLabel) {
      gridLabel.position.set(2.1, 0.1, 0.03);
      group.add(gridLabel);
    }
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

    const referenceSignature = String(sceneData.world_frame_id || "world");
    if (referenceSignature !== this._referenceSignature) {
      if (this.referenceGroup) {
        this.scene.remove(this.referenceGroup);
        disposeObject(this.referenceGroup);
      }
      this.referenceGroup = this._referenceObject(referenceSignature);
      this.scene.add(this.referenceGroup);
      this._referenceSignature = referenceSignature;
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
    this.selectedId = Number(id);
    const group = this.captureGroups.get(this.selectedId);
    if (!group) return;
    const box = new THREE.Box3().setFromObject(group);
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.8, 0.8);
    this.target.copy(center);
    this.camera.position.copy(center).add(new THREE.Vector3(radius, radius * 0.6, radius));
    this.camera.lookAt(this.target);
    this._setOrbitFromCamera();
    for (const item of this.captureGroups.values()) {
      item.scale.setScalar(item.userData.captureId === this.selectedId ? 1.03 : 1);
    }
    this._render();
  }

  frameAll() {
    if (!this.available) return;
    const box = new THREE.Box3().setFromObject(this.root);
    if (this.cameraGroup) box.expandByObject(this.cameraGroup);
    if (this.referenceGroup) box.expandByObject(this.referenceGroup);
    if (box.isEmpty()) {
      this.target.set(0, 0, 0);
      this.camera.position.set(4, 2.5, 3);
    } else {
      const center = box.getCenter(new THREE.Vector3());
      const radius = Math.max(box.getSize(new THREE.Vector3()).length() * 0.55, 1);
      this.target.copy(center);
      this.camera.position.copy(center).add(new THREE.Vector3(radius, radius * 0.6, radius));
    }
    this.camera.lookAt(this.target);
    this._setOrbitFromCamera();
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
    if (this.referenceGroup) disposeObject(this.referenceGroup);
    this._resizeObserver?.disconnect();
    if (this._onWindowResize) window.removeEventListener("resize", this._onWindowResize);
    this.renderer.dispose();
  }
}
