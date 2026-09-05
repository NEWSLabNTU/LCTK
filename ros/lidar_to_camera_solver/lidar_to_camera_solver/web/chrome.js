/** DOM chrome for the assisted-review page.
 *
 * Chrome deliberately has no network boundary.  The controller owns ``app``
 * (including selectedId and layers) and supplies callbacks for actions; this
 * module only reads that state and renders it into the approved page shell.
 */

const PARAMETER_NAMES = [
  "stability_window_s",
  "stability_max_translation_m",
  "stability_max_rotation_deg",
];

const PARAMETER_INPUTS = [
  "stability_window_s",
  "stability_max_translation_m",
  "stability_max_rotation_deg",
];

function finiteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function formatNumber(value, digits = 2, suffix = "") {
  const number = finiteNumber(value);
  return number == null ? "—" : `${number.toFixed(digits)}${suffix}`;
}

function formatCount(value) {
  const number = finiteNumber(value);
  return number == null ? "—" : String(Math.max(0, Math.round(number)));
}

function pairClass(rms) {
  const value = finiteNumber(rms);
  if (value == null) return "";
  return value > 45 ? "b" : value > 25 ? "m" : "g";
}

function pairId(value) {
  const number = Number(value);
  return Number.isSafeInteger(number) ? number : null;
}

function capturesById(scene) {
  return new Map(
    (scene && Array.isArray(scene.captures) ? scene.captures : [])
      .map((capture) => [pairId(capture.id), capture])
      .filter(([id]) => id != null),
  );
}

function cloudPointCount(cloud) {
  if (cloud instanceof ArrayBuffer) return Math.floor(cloud.byteLength / 12);
  if (ArrayBuffer.isView(cloud)) return Math.floor(cloud.byteLength / 12);
  return null;
}

function diversityMetric(diversity, key, digits, suffix) {
  if (!diversity) return { value: "—", fraction: 0, short: true };
  const value = finiteNumber(diversity[key]);
  const target = finiteNumber(
    diversity[`${key}_target`] ??
      diversity.targets?.[key] ??
      diversity.target?.[key],
  );
  const fraction = value == null || target == null || target <= 0
    ? 0
    : Math.max(0, Math.min(1, value / target));
  const short = Array.isArray(diversity.shortfalls) &&
    diversity.shortfalls.some((entry) => String(entry).toLowerCase().includes(key.replaceAll("_", " ")))
    || (target != null && value != null && value < target);
  return {
    value: value == null ? "—" : `${value.toFixed(digits)}${suffix}`,
    fraction,
    short,
    target: target == null ? null : `${target.toFixed(digits)}${suffix}`,
  };
}

function setText(element, value) {
  if (element) element.textContent = value == null ? "" : String(value);
}

function setClass(element, className) {
  if (!element) return;
  element.classList.remove("ok", "bad", "warn");
  if (className) element.classList.add(className);
}

/** Render the approved assisted-review chrome from controller-owned state. */
export class Chrome {
  constructor(root = typeof document === "undefined" ? null : document, callbacks = {}) {
    // The normal browser entry point is ``new Chrome(callbacks)``.  Accepting a
    // root document as the first argument keeps the module easy to embed and
    // test without changing that small public interface.
    if (!root || typeof root.querySelector !== "function") {
      callbacks = root || {};
      root = typeof document === "undefined" ? null : document;
    }
    this.root = root;
    this.callbacks = { ...callbacks };
    this._bound = false;
    this._sidebarCollapsed = false;
    this._previewKey = "";
    this._invalidSelection = null;
    this._autowareEntry = null;
    this._actionBusy = false;
    this._bind();
  }

  _query(selector) {
    return this.root?.querySelector(selector) || null;
  }

  _all(selector) {
    return this.root ? [...this.root.querySelectorAll(selector)] : [];
  }

  _callback(name, app) {
    const aliases = name === "onSetParams" ? ["onSetParams", "onParams"] : [name];
    const callback = aliases.map((key) => this.callbacks[key] || app?.callbacks?.[key])
      .find((candidate) => typeof candidate === "function");
    return typeof callback === "function" ? callback : null;
  }

  _call(name, app, ...args) {
    const callback = this._callback(name, app);
    if (!callback) return undefined;
    try {
      return callback(...args);
    } catch (error) {
      this._showActionError(error);
      return undefined;
    }
  }

  _bind() {
    if (this._bound) return;
    const menuButton = this._query("#menubtn");
    const menu = this._query("#menu");
    if (menuButton && menu) {
      menuButton.addEventListener("click", (event) => {
        event.stopPropagation();
        menu.classList.toggle("open");
      });
      this.root.addEventListener("click", (event) => {
        if (!menu.contains(event.target) && event.target !== menuButton) {
          menu.classList.remove("open");
        }
      });
    }

    const list = this._query("#list");
    if (list) {
      list.addEventListener("click", (event) => {
        const row = event.target.closest("[data-pair-id]");
        if (!row || !list.contains(row)) return;
        const id = pairId(row.dataset.pairId);
        if (id != null && this._app) this._call("onSelect", this._app, id);
      });
    }

    const close = this._query("#dClose");
    if (close) close.addEventListener("click", () => {
      if (this._app) this._call("onClose", this._app);
    });

    const drop = this._query(".dropbtn");
    if (drop) drop.addEventListener("click", () => {
      const id = pairId(this._app?.selectedId);
      if (id != null && this._app) this._call("onDrop", this._app, id);
    });

    const frameAll = this._query("#frameAll");
    if (frameAll) frameAll.addEventListener("click", () => {
      if (this._app) this._call("onFrameAll", this._app);
    });

    const handle = this._query("#handle");
    if (handle) handle.addEventListener("click", () => {
      this._sidebarCollapsed = !this._sidebarCollapsed;
      this._renderSidebarVisibility();
    });

    const layerControls = [
      ["#tPoints", "points"],
      ["#tFrustum", "frustum"],
      ["#tRms", "rms"],
    ];
    for (const [selector, layer] of layerControls) {
      const input = this._query(selector);
      if (input) input.addEventListener("change", () => {
        if (this._app) this._call("onLayer", this._app, layer, Boolean(input.checked));
      });
    }

    const items = this._all("#menu .item");
    if (items[0]) items[0].addEventListener("click", () => {
      const path = this._app?.state?.export?.archive_path || "";
      if (this._app) this._runAction("onExportArchive", this._app, path);
    });
    if (items[1]) items[1].addEventListener("click", () => {
      if (this._app) this._runAction("onAutowarePreview", this._app);
    });

    const apply = this._query(".apply");
    if (apply) apply.addEventListener("click", () => this._applyParams());
    this._bound = true;
  }

  _applyParams() {
    const app = this._app;
    if (!app) return;
    if (app.state?.params_writable === false) {
      this._showActionError(app.state.params_detail || "Parameter writes are disabled");
      return;
    }
    const values = {};
    const inputs = this._all(".param input");
    PARAMETER_INPUTS.forEach((name, index) => {
      const value = finiteNumber(inputs[index]?.value);
      if (value != null) values[name] = value;
    });
    if (Object.keys(values).length !== PARAMETER_NAMES.length) {
      this._showActionError("enter finite values for all three stability parameters");
      return;
    }
    this._runAction("onSetParams", app, values);
  }

  _renderSidebarVisibility() {
    const body = this._query("#bodyEl");
    const handle = this._query("#handle");
    if (body) body.classList.toggle("collapsed", this._sidebarCollapsed);
    if (handle) {
      handle.innerHTML = this._sidebarCollapsed ? "&#9654;" : "&#9664;";
      handle.title = this._sidebarCollapsed ? "Show sidebar" : "Hide sidebar";
    }
    const model = this._app?.model;
    if (model && typeof model._resize === "function" && typeof window !== "undefined") {
      window.setTimeout(() => model._resize(), 180);
    }
  }

  _showActionError(detail) {
    const box = this._query("#action-notice") || this._query("#autoware") || this._query("#scene-status");
    if (box) setText(box, detail instanceof Error ? detail.message : detail);
  }

  _showActionResult(result) {
    if (!result || typeof result !== "object") return;
    if (result.ok === false) {
      this._showActionError(result.detail || "request failed");
      return;
    }
    if (result.entry == null && result.detail == null) return;
    const box = result.entry != null
      ? this._query("#autoware") || this._query("#action-notice")
      : this._query("#action-notice") || this._query("#scene-status");
    if (!box) return;
    box.replaceChildren();
    if (result.entry != null) {
      this._autowareEntry = result.entry;
      const pre = document.createElement("pre");
      pre.textContent = JSON.stringify(result.entry, null, 2);
      box.append(pre);
      if (this._callback("onAutowareWrite", this._app)) {
        const confirm = document.createElement("button");
        confirm.type = "button";
        confirm.textContent = "Confirm write";
        confirm.addEventListener("click", () => {
          this._runAction("onAutowareWrite", this._app);
        });
        box.append(confirm);
      }
      return;
    }
    setText(box, result.detail || "ok");
  }

  _runAction(name, app, ...args) {
    const result = this._call(name, app, ...args);
    if (result && typeof result.then === "function") {
      this._actionBusy = true;
      this._renderParams(app);
      result.then((value) => this._showActionResult(value)).catch((error) => {
        this._showActionError(error);
      }).finally(() => {
        this._actionBusy = false;
        this._renderParams(app);
      });
    } else {
      this._showActionResult(result);
    }
    return result;
  }

  _renderList(app, pairs, captures) {
    const list = this._query("#list");
    if (!list) return;
    const selected = pairId(app.selectedId);
    list.replaceChildren();
    if (pairs.length === 0) {
      const empty = document.createElement("div");
      empty.className = "shortfall";
      empty.textContent = "No captures yet";
      list.append(empty);
      return;
    }
    for (const pair of pairs) {
      const id = pairId(pair.id);
      if (id == null) continue;
      const row = document.createElement("div");
      row.className = `row ${pairClass(pair.rms_px)}${selected === id ? " sel" : ""}`;
      row.dataset.pairId = String(id);
      row.tabIndex = 0;
      row.setAttribute("role", "button");
      row.setAttribute("aria-pressed", String(selected === id));

      const top = document.createElement("div");
      top.className = "top";
      const idLabel = document.createElement("span");
      idLabel.className = "id";
      idLabel.textContent = `#${id}`;
      const pill = document.createElement("span");
      pill.className = "pill";
      pill.textContent = formatNumber(pair.rms_px, 1, " px");
      top.append(idLabel, pill);

      const meta = document.createElement("div");
      meta.className = "meta";
      const capture = captures.get(id);
      const position = capture?.position;
      const range = Array.isArray(position) && position.length >= 3
        ? Math.hypot(Number(position[0]), Number(position[1]), Number(position[2]))
        : null;
      const inliers = cloudPointCount(app.clouds?.get(id));
      const parts = [];
      if (range != null && Number.isFinite(range)) parts.push(`range ${range.toFixed(2)} m`);
      if (inliers != null) parts.push(`${formatCount(inliers)} inliers`);
      if (pair.missing?.length) parts.push(`missing ${pair.missing.join(", ")}`);
      meta.textContent = parts.length ? parts.join(" · ") : "evidence unavailable";
      row.append(top, meta);
      row.addEventListener("keydown", (event) => {
        if (event.key !== "Enter" && event.key !== " ") return;
        event.preventDefault();
        this._call("onSelect", app, id);
      });
      list.append(row);
    }
  }

  _renderDetail(app, pair, capture) {
    const detail = this._query("#detail");
    const divider = this._query("#divider");
    const preview = this._query("#preview");
    const drop = this._query(".dropbtn");
    if (!detail || !divider) return;
    const open = pair != null;
    detail.classList.toggle("open", open);
    divider.classList.toggle("open", open);
    if (!open) {
      this._previewKey = "";
      if (preview) preview.hidden = true;
      return;
    }
    const id = pairId(pair.id);
    setText(this._query("#dTitle"), `pair #${id}`);
    setText(this._query("#kRms"), formatNumber(pair.rms_px, 1, " px"));
    const position = capture?.position;
    const range = Array.isArray(position) && position.length >= 3
      ? Math.hypot(Number(position[0]), Number(position[1]), Number(position[2]))
      : null;
    setText(this._query("#kRange"), formatNumber(range, 2, " m"));
    const inliers = cloudPointCount(app.clouds?.get(id));
    const kv = this._all("#detail .kv b");
    if (kv[2]) setText(kv[2], inliers == null ? "unavailable" : formatCount(inliers));
    const markerCount = Array.isArray(capture?.marker_quads_world)
      ? capture.marker_quads_world.length
      : null;
    const expectedMarkers = finiteNumber(app.scene?.marker_count);
    if (kv[3]) {
      setText(kv[3], markerCount == null
        ? "unavailable"
        : `${formatCount(markerCount)}${expectedMarkers == null ? "" : ` / ${formatCount(expectedMarkers)}`}`);
    }
    const stamp = pair.stamp_s ?? pair.timestamp_s ?? pair.timestamp;
    if (kv[4]) setText(kv[4], stamp == null ? "not reported" : `t ${formatNumber(stamp, 3, " s")}`);
    if (drop) drop.disabled = this._actionBusy;

    const missing = Array.isArray(pair.missing) ? pair.missing : [];
    let host = this._query("#preview-host");
    if (!host && preview) {
      host = document.createElement("div");
      host.id = "preview-host";
      host.className = "preview-host";
      preview.hidden = true;
      preview.parentElement?.insertBefore(host, preview);
    }
    if (!host) return;
    const revision = finiteNumber(app.state?.scene_revision) ?? 0;
    const hasPreview = pair.has_preview === true && !missing.includes("camera frame");
    const previewState = app.preview?.id === id
      ? app.preview
      : { status: "idle", url: null };
    const key = `${id}:${revision}:${hasPreview ? "yes" : "no"}:${previewState.status}:${previewState.url || ""}`;
    if (key === this._previewKey) return;
    this._previewKey = key;
    host.replaceChildren();
    if (!hasPreview) {
      const message = document.createElement("div");
      message.className = "shortfall";
      message.textContent = missing.length
        ? `No matching evidence: ${missing.join(", ")}`
        : "No matching camera frame";
      host.append(message);
      return;
    }
    if (previewState.status !== "ready" || !previewState.url) {
      const loading = document.createElement("div");
      loading.className = "shortfall";
      loading.textContent = previewState.status === "missing"
        ? "Preview unavailable"
        : "Loading preview…";
      host.append(loading);
      return;
    }
    const image = document.createElement("img");
    image.className = "preview-image";
    image.alt = `ArUco preview for pair #${id}`;
    image.style.display = "block";
    image.style.width = "100%";
    image.style.borderRadius = "6px";
    image.src = previewState.url;
    const renderedKey = key;
    const loading = document.createElement("div");
    loading.className = "shortfall";
    loading.textContent = "Loading preview…";
    image.addEventListener("load", () => {
      if (this._previewKey === renderedKey) loading.remove();
    }, { once: true });
    image.addEventListener("error", () => {
      if (this._previewKey !== renderedKey) return;
      host.replaceChildren();
      const message = document.createElement("div");
      message.className = "shortfall";
      message.textContent = "Preview unavailable";
      host.append(message);
    }, { once: true });
    host.append(loading, image);
  }

  _renderFooter(app, pairs) {
    const state = app.state || {};
    const diversity = state.diversity || {};
    const gauges = this._all("footer .gauge");
    const placementTarget = finiteNumber(
      diversity.n_placements_target ?? diversity.targets?.n_placements,
    );
    const placementValue = finiteNumber(diversity.n_placements);
    const metrics = [
      ["Placements", {
        value: formatCount(placementValue),
        fraction: placementValue == null || placementTarget == null || placementTarget <= 0
          ? 0
          : Math.max(0, Math.min(1, placementValue / placementTarget)),
        target: placementTarget,
      }, Boolean(diversity.is_degenerate)],
      ["Normal span", diversityMetric(diversity, "normal_span_deg", 1, "°")],
      ["Depth range", diversityMetric(diversity, "depth_range_m", 2, " m")],
      ["Lateral span", diversityMetric(diversity, "lateral_span_m", 2, " m")],
    ];
    gauges.forEach((gauge, index) => {
      const metric = metrics[index];
      if (!metric) return;
      const label = gauge.querySelector(".label");
      const value = label?.querySelector("b");
      const track = gauge.querySelector(".track");
      const fill = track?.querySelector("i");
      if (index === 0) {
        setText(label?.querySelector("span"), metric[0]);
        const target = metric[1].target == null ? null : formatCount(metric[1].target);
        setText(value, target == null ? metric[1].value : `${metric[1].value} / ${target}`);
        if (fill) fill.style.width = `${metric[1].fraction * 100}%`;
        track?.classList.toggle("short", metric[2] || (target != null && metric[1].fraction < 1));
      } else {
        setText(label?.querySelector("span"), metric[0]);
        setText(value, metric[1].target == null ? metric[1].value : `${metric[1].value} / ${metric[1].target}`);
        if (fill) fill.style.width = `${metric[1].fraction * 100}%`;
        track?.classList.toggle("short", metric[1].short);
      }
    });

    const cells = this._all("footer .cell");
    const stillness = state.stillness || {};
    const stillValue = cells[4]?.querySelector(".value");
    setClass(stillValue, stillness.is_still ? "ok" : "bad");
    if (stillValue) {
      const dot = document.createElement("span");
      dot.className = "dot";
      stillValue.replaceChildren(dot, document.createTextNode(
        `${stillness.is_still ? "still" : "moving"} — ${stillness.reason || "waiting"}`,
      ));
    }
    const solve = state.solve || {};
    const solveValue = cells[5]?.querySelector(".value");
    const solved = String(solve.status || "").toLowerCase().startsWith("solved");
    setClass(solveValue, solved ? "ok" : "bad");
    setText(solveValue, `${solve.status || "unknown"}${solve.rms_px == null ? "" : ` — RMS ${formatNumber(solve.rms_px, 1, " px")}`}`);
    const syncValue = cells[6]?.querySelector(".value");
    setText(syncValue, state.sync || "waiting for synchronization");

    const legendSubs = this._all(".legend .sub");
    const cloudPoints = pairs.reduce((total, pair) => {
      const count = cloudPointCount(app.clouds?.get(pairId(pair.id)));
      return total + (count == null ? 0 : count);
    }, 0);
    setText(legendSubs[0], `${pairs.length} pair${pairs.length === 1 ? "" : "s"} · ${cloudPoints} inliers`);
    setText(legendSubs[1], "LMB pan · RMB orbit · wheel zoom");
  }

  _renderParams(app) {
    const inputs = this._all(".param input");
    const params = app.state?.params || app.state?.stability_params || {};
    const values = PARAMETER_NAMES.map((name) => params[name]);
    inputs.forEach((input, index) => {
      if (document.activeElement === input || values[index] == null) return;
      const value = finiteNumber(values[index]);
      if (value != null) input.value = value.toFixed(3);
    });
    const apply = this._query(".apply");
    const caution = this._query("#paramCaution");
    const writable = app.state?.params_writable;
    if (caution) {
      setText(
        caution,
        writable === false && app.state?.params_detail
          ? app.state.params_detail
          : "Applies to future captures. Already-buffered pairs keep the gate they were taken under.",
      );
    }
    if (apply) {
      apply.disabled = writable === false || this._actionBusy;
      apply.title = writable === false
        ? (app.state?.params_detail || "Parameter writes are disabled")
        : "Apply to future captures";
    }
  }

  /** Render all DOM owned by Chrome.  Callbacks are the only write path. */
  render(app = {}) {
    this._app = app;
    this._bind();
    const state = app.state || {};
    const pairs = Array.isArray(state.pairs) ? state.pairs : [];
    const captures = capturesById(app.scene);
    const selected = pairId(app.selectedId);
    const selectedPair = pairs.find((pair) => pairId(pair.id) === selected) || null;
    if (selected != null && !selectedPair && this._invalidSelection !== selected) {
      this._invalidSelection = selected;
      this._call("onClose", app);
    } else if (selectedPair) {
      this._invalidSelection = null;
    }
    this._renderList(app, pairs, captures);
    this._renderDetail(app, selectedPair, captures.get(selected));
    this._renderFooter(app, pairs);
    this._renderParams(app);

    const notice = this._query("#action-notice");
    if (notice && app.notice) setText(notice, app.notice);

    const status = this._query("#scene-status");
    if (status && app.model?.error) setText(status, `3D viewport unavailable: ${app.model.error}`);
    const autoware = this._query("#autoware");
    if (autoware && this._autowareEntry && !autoware.textContent) {
      setText(autoware, JSON.stringify(this._autowareEntry, null, 2));
    }
    return this;
  }
}

export default Chrome;
