import { Chrome } from "./chrome.js";
import { ReviewApi } from "./review_api.js";
import { ReviewSession } from "./review_session.js";

const canvas = document.getElementById("scene") || document.getElementById("view");
if (canvas) {
  const api = new ReviewApi();
  let session;
  let chrome;
  let model;
  let sceneLoadStarted = false;

  function render(dirty = { all: true }) {
    if (dirty.all || dirty.chrome !== false) chrome?.render(session.app, dirty);
    if (dirty.all || dirty.scene !== false) model?.sync(session.app, dirty);
  }

  function loadSceneModel() {
    if (sceneLoadStarted) return;
    sceneLoadStarted = true;
    void import("./scene_model.js").then(({ SceneModel }) => {
      model = new SceneModel(canvas, {
        onPick: (id) => session?.select(id),
      });
      session.app.model = model;
      window.lctkScene.model = model;
      render({ chrome: true, scene: true });
      if (session.app.selectedId != null) model.focus(session.app.selectedId);
    }).catch((error) => {
      session.app.model = { error: error instanceof Error ? error.message : String(error) };
      render({ chrome: true, scene: false });
    });
  }

  session = new ReviewSession(api, {
    onChange: (dirty) => {
      render(dirty);
      // Let Chrome paint the first state before parsing/initialising Three.js.
      if (session.app.state) loadSceneModel();
    },
    onFocus: (id) => model?.focus(id),
    onFrameAll: () => model?.frameAll(),
  });

  chrome = new Chrome({
    onSelect: (id) => session.select(id),
    onClose: () => session.clearSelection(),
    onFrameAll: () => model?.frameAll(),
    onLayer: (name, enabled) => {
      if (name in session.app.layers) session.app.layers[name] = Boolean(enabled);
      render({ chrome: true, scene: true });
    },
    onDrop: (id) => session.drop(id),
    onExportArchive: (path) => api.exportArchive(path),
    onAutowarePreview: async () => {
      const result = await api.autowarePreview();
      session.app.autowarePreview = result.ok ? result.entry : null;
      return result;
    },
    onAutowareWrite: () => api.autowareWrite(),
    onSetParams: (values) => api.setParams(values),
  });

  window.lctkScene = {
    app: session.app,
    session,
    model,
    setLayer(name, enabled) {
      if (name in session.app.layers) session.app.layers[name] = Boolean(enabled);
      render({ chrome: true, scene: true });
    },
    select: (id) => session.select(id),
    clearSelection: () => session.clearSelection(),
    frameAll: () => model?.frameAll(),
  };

  render({ all: true });
  setInterval(() => session.poll(), 500);

  // Start the cheap state request before loading the vendored Three.js module.
  // The first list/footer paint should not wait for WebGL parsing or context
  // creation. Scene hydration remains independent and fills in when ready.
  void session.start();
}
