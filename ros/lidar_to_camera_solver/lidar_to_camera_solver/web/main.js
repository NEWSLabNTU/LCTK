import { ReviewApi } from "./review_api.js";
import { SceneModel } from "./scene_model.js";

const canvas = document.getElementById("scene");
if (canvas) {
  const api = new ReviewApi();
  const status = document.getElementById("scene-status");
  const app = {
    api,
    state: null,
    scene: { scene_revision: -1, captures: [], camera: null },
    clouds: new Map(),
    selectedId: null,
    layers: { points: true, frustum: true, rms: true },
  };
  const model = new SceneModel(canvas, {
    onPick: (id) => {
      app.selectedId = id;
      if (status) status.textContent = `selected pair #${id}`;
      window.dispatchEvent(new CustomEvent("lctk-scene-selection", { detail: id }));
    },
  });
  let sceneRevision = -1;
  let requestSerial = 0;
  let polling = false;

  async function syncScene(state) {
    if (state.scene_revision === sceneRevision && app.scene) return;
    const serial = ++requestSerial;
    const scene = await api.scene();
    if (serial !== requestSerial || !scene || scene.ok === false) return;
    const captures = scene.captures || [];
    const cloudEntries = await Promise.all(
      captures.map(async (capture) => [Number(capture.id), await api.cloud(capture.id)]),
    );
    if (serial !== requestSerial) return;
    app.scene = scene;
    for (const [id, cloud] of cloudEntries) app.clouds.set(id, cloud);
    sceneRevision = Number(scene.scene_revision);
  }

  async function poll() {
    if (polling) return;
    polling = true;
    try {
      const state = await api.state();
      if (!state || state.ok === false) {
        if (status) status.textContent = `scene unavailable: ${state?.detail || "request failed"}`;
        return;
      }
      app.state = state;
      await syncScene(state);
      model.sync(app);
      if (status && model.error) status.textContent = `3D viewport unavailable: ${model.error}`;
    } finally {
      polling = false;
    }
  }

  window.lctkScene = {
    app,
    model,
    setLayer(name, enabled) {
      if (name in app.layers) app.layers[name] = Boolean(enabled);
      model.sync(app);
    },
    select(id) {
      app.selectedId = Number(id);
      model.focus(app.selectedId);
    },
    frameAll() {
      model.frameAll();
    },
  };
  setInterval(poll, 500);
  poll();
}
