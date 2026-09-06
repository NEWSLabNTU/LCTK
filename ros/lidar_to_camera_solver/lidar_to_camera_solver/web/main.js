import { Chrome } from "./chrome.js";
import { ReviewApi } from "./review_api.js";
import { SceneModel } from "./scene_model.js";

const canvas = document.getElementById("scene") || document.getElementById("view");
if (canvas) {
  const api = new ReviewApi();
  const app = {
    api,
    state: null,
    scene: { scene_revision: -1, captures: [], camera: null },
    clouds: new Map(),
    selectedId: null,
    selectedCaptureId: null,
    preview: { id: null, url: null, status: "idle" },
    notice: "",
    layers: { points: true, frustum: true, rms: true },
  };
  let previewSerial = 0;
  let sceneRevision = -1;
  let sceneRequestSerial = 0;
  let polling = false;

  const model = new SceneModel(canvas, {
    onPick: (id) => select(id),
  });

  function render() {
    chrome.render(app);
    model.sync(app);
  }

  async function loadPreview(id) {
    const serial = ++previewSerial;
    if (app.preview.url) URL.revokeObjectURL(app.preview.url);
    app.preview = { id, url: null, status: "loading" };
    render();
    const blob = await api.preview(id);
    if (serial !== previewSerial || app.selectedId !== id) return;
    if (blob) {
      app.preview = { id, url: URL.createObjectURL(blob), status: "ready" };
    } else {
      app.preview = { id, url: null, status: "missing" };
    }
    render();
  }

  async function retryMissingPreview() {
    if (app.selectedId == null || app.preview.status !== "missing") return;
    await loadPreview(app.selectedId);
  }

  function select(id) {
    const numericId = Number(id);
    const exists = (app.state?.pairs || []).some(
      (pair) => Number(pair.id) === numericId,
    );
    if (!exists) return clearSelection();
    app.selectedId = numericId;
    app.selectedCaptureId = numericId;
    model.focus(numericId);
    loadPreview(numericId);
    render();
  }

  function clearSelection() {
    previewSerial += 1;
    if (app.preview.url) URL.revokeObjectURL(app.preview.url);
    app.selectedId = null;
    app.selectedCaptureId = null;
    app.preview = { id: null, url: null, status: "idle" };
    render();
    model.frameAll();
  }

  function pruneCloudCaches() {
    const pairIds = new Set(
      (app.state?.pairs || [])
        .map((pair) => Number(pair.id))
        .filter((id) => Number.isFinite(id)),
    );
    const sceneIds = new Set(
      (app.scene?.captures || [])
        .map((capture) => Number(capture.id))
        .filter((id) => Number.isFinite(id)),
    );
    const activeIds = new Set([...pairIds].filter((id) => sceneIds.has(id)));
    for (const id of app.clouds.keys()) {
      if (!activeIds.has(id)) app.clouds.delete(id);
    }
    api.pruneClouds(activeIds);
  }

  async function syncScene(state) {
    if (Number(state.scene_revision) === sceneRevision && app.scene) return;
    const serial = ++sceneRequestSerial;
    const scene = await api.scene();
    if (serial !== sceneRequestSerial || !scene || scene.ok === false) return;
    app.scene = scene;
    sceneRevision = Number(scene.scene_revision);
  }

  async function syncMissingClouds() {
    const captures = app.scene?.captures || [];
    const missing = captures.filter((capture) => {
      const id = Number(capture.id);
      return Number.isFinite(id) && (!app.clouds.has(id) || app.clouds.get(id) == null);
    });
    const cloudEntries = await Promise.all(
      missing.map(async (capture) => [
        Number(capture.id),
        await api.cloud(capture.id),
      ]),
    );
    for (const [id, cloud] of cloudEntries) {
      if (cloud != null) app.clouds.set(id, cloud);
    }
  }

  const chrome = new Chrome({
    onSelect: select,
    onClose: clearSelection,
    onFrameAll: () => model.frameAll(),
    onLayer: (name, enabled) => {
      if (name in app.layers) app.layers[name] = Boolean(enabled);
      render();
    },
    onDrop: async (id) => {
      const result = await api.drop(id);
      if (result.ok && app.selectedId === Number(id)) clearSelection();
      return result;
    },
    onExportArchive: (path) => api.exportArchive(path),
    onAutowarePreview: async () => {
      const result = await api.autowarePreview();
      app.autowarePreview = result.ok ? result.entry : null;
      return result;
    },
    onAutowareWrite: () => api.autowareWrite(),
    onSetParams: (values) => api.setParams(values),
  });

  async function poll() {
    if (polling) return;
    polling = true;
    try {
      const state = await api.state();
      if (!state || state.ok === false) {
        app.notice = `server unavailable: ${state?.detail || "request failed"}`;
        render();
        return;
      }
      app.state = state;
      const ids = new Set((state.pairs || []).map((pair) => Number(pair.id)));
      if (app.selectedId != null && !ids.has(app.selectedId)) clearSelection();
      await syncScene(state);
      await syncMissingClouds();
      await retryMissingPreview();
      pruneCloudCaches();
      render();
    } finally {
      polling = false;
    }
  }

  window.lctkScene = {
    app,
    model,
    setLayer(name, enabled) {
      if (name in app.layers) app.layers[name] = Boolean(enabled);
      render();
    },
    select,
    frameAll: () => model.frameAll(),
  };
  setInterval(poll, 500);
  poll();
}
