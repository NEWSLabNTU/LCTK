/** DOM chrome for the assisted-review page.
 *
 * Chrome deliberately has no network boundary.  The controller owns ``app``
 * (including selectedId and layers) and supplies callbacks for actions; this
 * module only reads that state and renders it into the approved page shell.
 */

import { rmsBand } from "./quality.js";

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
  const band = rmsBand(rms);
  return band === "high" ? "b" : band === "medium" ? "m" : band === "low" ? "g" : "";
}

// Cloud payloads are immutable once they enter ReviewSession.  Revisions are
// the normal invalidation signal; the identity token catches a replacement
// buffer from a small embedder that has not assigned a revision yet, without
// scanning a potentially large point cloud on every heartbeat.
const cloudObjectIds = new WeakMap();
let nextCloudObjectId = 1;

function cloudToken(cloud) {
  if (!cloud || typeof cloud !== "object") return null;
  let token = cloudObjectIds.get(cloud);
  if (token == null) {
    token = nextCloudObjectId;
    nextCloudObjectId += 1;
    cloudObjectIds.set(cloud, token);
  }
  return token;
}

function valueKey(value) {
  return JSON.stringify(value);
}

function cloudEntries(app) {
  const entries = new Map();
  const revisions = app?.cloudRevisions;
  const clouds = app?.clouds;
  const add = (id, revision, cloud) => {
    const number = pairId(id);
    if (number == null) return;
    entries.set(number, {
      revision: revision == null ? null : Number(revision),
      token: cloudToken(cloud),
    });
  };
  if (revisions instanceof Map) {
    for (const [id, revision] of revisions) add(id, revision, cloudFor(clouds, pairId(id)));
  } else if (revisions && typeof revisions === "object") {
    for (const [id, revision] of Object.entries(revisions)) add(id, revision, cloudFor(clouds, pairId(id)));
  }
  if (clouds instanceof Map) {
    for (const [id, cloud] of clouds) {
      const number = pairId(id);
      if (number == null || entries.has(number)) continue;
      add(number, null, cloud);
    }
  } else if (clouds && typeof clouds === "object") {
    for (const [id, cloud] of Object.entries(clouds)) {
      const number = pairId(id);
      if (number == null || entries.has(number)) continue;
      add(number, null, cloud);
    }
  }
  return [...entries.entries()].sort((left, right) => left[0] - right[0]);
}

function cloudFor(app, id) {
  if (app?.clouds instanceof Map) return app.clouds.get(id);
  if (app?.clouds && typeof app.clouds === "object") return app.clouds[id];
  return null;
}

export function pairId(value) {
  if (value == null || typeof value === "boolean") return null;
  if (typeof value === "string" && value.trim() === "") return null;
  if (typeof value !== "number" && typeof value !== "string") return null;
  const number = Number(value);
  return Number.isSafeInteger(number) && number >= 0 ? number : null;
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
  if (!element) return;
  const text = value == null ? "" : String(value);
  if (element.textContent !== text) element.textContent = text;
}

function setClass(element, className) {
  if (!element) return;
  const classes = ["ok", "bad", "warn"];
  const current = classes.find((name) => element.classList.contains(name)) || null;
  if (current === className) return;
  element.classList.remove(...classes);
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
    this._autowareAvailabilityText = "";
    this._actionBusy = false;
    this._paramDraft = null;
    this._paramDirty = false;
    this._paramDirtyNames = new Set();
    this._paramPendingEffective = null;
    this._listRenderKey = null;
    this._detailRenderKey = null;
    this._footerGaugeRenderKey = null;
    this._footerLiveRenderKey = null;
    this._footerLegendRenderKey = null;
    this._paramsRenderKey = null;
    this._exportRenderKey = null;
    this._rows = new Map();
    this._emptyListRow = null;
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
      const path = (this._app?.live || this._app?.state)?.export?.archive_path || "";
      if (this._app) this._runAction("onExportArchive", this._app, path);
    });
    if (items[1]) items[1].addEventListener("click", () => {
      if (!this._app) return;
      const availability = this._autowareAvailability(this._app);
      if (!availability.available) {
        this._showActionError(availability.reason);
        return;
      }
      this._runAction("onAutowarePreview", this._app);
    });

    const apply = this._query(".apply");
    if (apply) apply.addEventListener("click", () => this._applyParams());
    const reset = this._query("#paramReset");
    if (reset) reset.addEventListener("click", () => {
      this._paramDraft = null;
      this._paramDirty = false;
      this._paramDirtyNames.clear();
      this._paramPendingEffective = null;
      if (this._app) this._renderParams(this._app);
    });
    for (const [index, input] of this._all(".param input").entries()) {
      input.addEventListener("input", () => {
        this._paramDraft = this._readParamDraft();
        this._paramDirty = true;
        this._paramDirtyNames.add(PARAMETER_NAMES[index]);
        this._paramPendingEffective = null;
        if (this._app) this._renderParams(this._app);
      });
    }
    this._bound = true;
  }

  _readParamDraft() {
    const inputs = this._all(".param input");
    return Object.fromEntries(
      PARAMETER_INPUTS.map((name, index) => [name, inputs[index]?.value ?? ""]),
    );
  }

  _normaliseParamValues(values) {
    return Object.fromEntries(
      PARAMETER_NAMES.map((name) => {
        const value = finiteNumber(values?.[name]);
        return [name, value == null ? "" : String(value)];
      }),
    );
  }

  _writeParamDraft(draft) {
    const inputs = this._all(".param input");
    PARAMETER_INPUTS.forEach((name, index) => {
      const value = draft?.[name] ?? "";
      if (inputs[index] && inputs[index].value !== value) inputs[index].value = value;
    });
  }

  _sameParamDraft(left, right) {
    return PARAMETER_NAMES.every(
      (name) => String(left?.[name] ?? "") === String(right?.[name] ?? ""),
    );
  }

  _autowareAvailability(app) {
    const live = app.live || app.state || {};
    const explicit = live.export_availability?.autoware;
    const ready = explicit?.available ?? live.export?.autoware_ready;
    const missing = explicit?.missing
      ?? live.export?.autoware_missing
      ?? [];
    const reason = explicit?.reason
      || (ready
        ? "ready"
        : (missing.length
          ? `unset parameter(s): ${missing.join(", ")}`
          : "Autoware export is unavailable"));
    return {
      available: ready === true,
      reason,
      missing,
    };
  }

  _applyParams() {
    const app = this._app;
    if (!app) return;
    const live = app.live || app.state || {};
    if (live.params_writable === false) {
      this._showActionError(live.params_detail || "Parameter writes are disabled");
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
    if (
      model
      && !model._resizeObserver
      && typeof model._resize === "function"
      && typeof window !== "undefined"
    ) {
      window.setTimeout(() => model._resize(), 180);
    }
  }

  _showActionError(detail) {
    const box = this._query("#action-notice")
      || this._query("#autoware")
      || this._query("#scene-status");
    if (box) {
      const message = detail instanceof Error ? detail.message : detail;
      setText(box, message);
      box.title = String(message ?? "");
    }
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
    const submittedDraft = name === "onSetParams" ? this._readParamDraft() : null;
    const result = this._call(name, app, ...args);
    if (result && typeof result.then === "function") {
      this._actionBusy = true;
      this._renderParams(app);
      result.then((value) => {
        if (name === "onSetParams" && value?.ok === true) {
          const currentDraft = this._readParamDraft();
          if (this._sameParamDraft(currentDraft, submittedDraft)) {
            const effective = value.params
              || app.state?.params
              || app.state?.stability_params;
            if (effective) {
              this._paramDraft = this._normaliseParamValues(effective);
              this._paramDirty = false;
              this._paramDirtyNames.clear();
              this._paramPendingEffective = this._paramDraft;
            }
          }
        }
        this._showActionResult(value);
      }).catch((error) => {
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

  _createElement(tagName) {
    const owner = this.root?.ownerDocument;
    if (owner?.createElement) return owner.createElement(tagName);
    if (typeof document !== "undefined" && document.createElement) {
      return document.createElement(tagName);
    }
    return null;
  }

  _buildListRow(app, id) {
    const row = this._createElement("div");
    if (!row) return null;
    row.className = "row";
    row.dataset.pairId = String(id);
    row.tabIndex = 0;
    row.setAttribute("role", "button");
    const top = this._createElement("div");
    const idLabel = this._createElement("span");
    const pill = this._createElement("span");
    const meta = this._createElement("div");
    if (!top || !idLabel || !pill || !meta) return null;
    top.className = "top";
    idLabel.className = "id";
    // The capture id is immutable for the lifetime of a keyed row.  Keeping
    // this write in the constructor path avoids touching static labels on
    // every heartbeat.
    idLabel.textContent = `#${id}`;
    pill.className = "pill";
    pill.title = "Recomputed for all captures after calibration changes";
    meta.className = "meta";
    top.append(idLabel, pill);
    row.append(top, meta);
    row._chromeParts = { pill, meta };
    row.addEventListener("keydown", (event) => {
      if (event.key !== "Enter" && event.key !== " ") return;
      event.preventDefault();
      this._call("onSelect", this._app || app, id);
    });
    return row;
  }

  _updateListRow(app, row, pair, capture, selected) {
    const id = pairId(pair.id);
    if (id == null || !row) return;
    const className = `row ${pairClass(pair.rms_px)}${selected === id ? " sel" : ""}`;
    if (row.className !== className) row.className = className;
    if (row.getAttribute?.("aria-pressed") !== String(selected === id)) {
      row.setAttribute("aria-pressed", String(selected === id));
    }
    const position = capture?.position;
    const range = Array.isArray(position) && position.length >= 3
      ? Math.hypot(Number(position[0]), Number(position[1]), Number(position[2]))
      : null;
    const inliers = cloudPointCount(cloudFor(app, id));
    const parts = [];
    if (range != null && Number.isFinite(range)) parts.push(`range ${range.toFixed(2)} m`);
    if (inliers != null) parts.push(`${formatCount(inliers)} inliers`);
    if (pair.missing?.length) parts.push(`missing ${pair.missing.join(", ")}`);
    const metaText = parts.length ? parts.join(" · ") : "evidence unavailable";
    const rowParts = row._chromeParts || {};
    setText(rowParts.pill, formatNumber(pair.rms_px, 1, " px"));
    setText(rowParts.meta, metaText);
  }

  _renderList(app, pairs, captures) {
    const summary = this._query("#detSummary");
    const validPairs = pairs
      .map((pair) => [pairId(pair.id), pair])
      .filter(([id]) => id != null);
    if (summary) {
      if (app.state == null) {
        setText(summary, "loading…");
      } else {
        const count = validPairs.length;
        setText(summary, `${count} capture${count === 1 ? "" : "s"}`);
      }
    }
    const list = this._query("#list");
    if (!list) return;
    this._rows ||= new Map();
    const selected = pairId(app.selectedId);
    const desired = [];
    for (const [id, pair] of validPairs) {
      let row = this._rows.get(id);
      if (!row) {
        row = this._buildListRow(app, id);
        if (!row) continue;
        this._rows.set(id, row);
      }
      this._updateListRow(app, row, pair, captures.get(id), selected);
      desired.push(row);
    }

    const desiredSet = new Set(desired);
    for (const [id, row] of this._rows) {
      if (desiredSet.has(row)) continue;
      row.remove?.();
      this._rows.delete(id);
    }

    if (desired.length === 0) {
      if (!this._emptyListRow) {
        this._emptyListRow = this._createElement("div");
        if (this._emptyListRow) {
          this._emptyListRow.className = "shortfall";
          this._emptyListRow.textContent = "No captures yet";
        }
      }
      const empty = this._emptyListRow;
      if (empty && list.children?.[0] !== empty) list.insertBefore(empty, list.children?.[0] || null);
      for (const child of [...(list.children || [])]) {
        if (child !== empty) child.remove?.();
      }
      return;
    }

    if (this._emptyListRow) this._emptyListRow.remove?.();
    // Move a row only when its position in the keyed order is wrong.  This
    // preserves focus, hover state, and the event listener for unchanged rows.
    desired.forEach((row, index) => {
      if (list.children?.[index] !== row) list.insertBefore(row, list.children?.[index] || null);
    });
    for (const child of [...(list.children || [])]) {
      if (!desiredSet.has(child)) child.remove?.();
    }
  }

  _renderDetail(app, pair, capture) {
    const detail = this._query("#detail");
    const divider = this._query("#divider");
    const drop = this._query(".dropbtn");
    if (!detail || !divider) return;
    const open = pair != null;
    detail.classList.toggle("open", open);
    divider.classList.toggle("open", open);
    if (!open) {
      this._previewKey = "";
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
    setText(this._query("#kInliers"), inliers == null ? "unavailable" : formatCount(inliers));
    const markerCount = Array.isArray(capture?.marker_quads_world)
      ? capture.marker_quads_world.length
      : null;
    const expectedMarkers = finiteNumber(app.scene?.marker_count);
    setText(this._query("#kMarkers"), markerCount == null
      ? "unavailable"
      : `${formatCount(markerCount)}${expectedMarkers == null ? "" : ` / ${formatCount(expectedMarkers)}`}`);
    const stamp = pair.stamp_s ?? pair.timestamp_s ?? pair.timestamp;
    setText(this._query("#kCaptured"), stamp == null ? "not reported" : `t ${formatNumber(stamp, 3, " s")}`);
    if (drop) drop.disabled = this._actionBusy;

    const missing = Array.isArray(pair.missing) ? pair.missing : [];
    let host = this._query("#preview-host");
    if (!host) {
      host = document.createElement("div");
      host.id = "preview-host";
      host.className = "preview-host";
      const inner = this._query("#detail .inner");
      const metrics = inner?.querySelector(".kv");
      if (inner) inner.insertBefore(host, metrics || inner.firstChild);
    }
    if (!host) return;
    const revision = finiteNumber(pair.evidence_revision)
      ?? finiteNumber(app.state?.scene_revision)
      ?? 0;
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

  _renderFooterGauges(app) {
    const state = app.captures || app.state || {};
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
        const target = metric[1].target == null ? null : formatCount(metric[1].target);
        setText(value, target == null ? metric[1].value : `${metric[1].value} / ${target}`);
        if (fill) fill.style.width = `${metric[1].fraction * 100}%`;
        track?.classList.toggle("short", metric[2] || (target != null && metric[1].fraction < 1));
      } else {
        setText(value, metric[1].target == null ? metric[1].value : `${metric[1].value} / ${metric[1].target}`);
        if (fill) fill.style.width = `${metric[1].fraction * 100}%`;
        track?.classList.toggle("short", metric[1].short);
      }
    });
  }

  _renderFooterLive(app) {
    const live = app.live || app.state || {};
    const captures = app.captures || app.state || {};
    const cells = this._all("footer .cell");
    const stillness = live.stillness || {};
    const stillValue = cells[4]?.querySelector(".value");
    setClass(stillValue, stillness.is_still ? "ok" : "bad");
    if (stillValue) {
      const text = `${stillness.is_still ? "still" : "moving"} — ${stillness.reason || "waiting"}`;
      const existingDot = stillValue.querySelector(".dot");
      if (!existingDot || stillValue.textContent !== text) {
        const dot = document.createElement("span");
        dot.className = "dot";
        stillValue.replaceChildren(dot, document.createTextNode(text));
      }
    }
    const solve = captures.solve || {};
    const solveValue = cells[5]?.querySelector(".value");
    const solved = String(solve.status || "").toLowerCase().startsWith("solved");
    setClass(solveValue, solved ? "ok" : "bad");
    setText(solveValue, `${solve.status || "unknown"}${solve.rms_px == null ? "" : ` — RMS ${formatNumber(solve.rms_px, 1, " px")}`}`);
    const syncValue = cells[6]?.querySelector(".value");
    setText(syncValue, live.sync || "waiting for synchronization");
  }

  _renderFooterLegend(app, pairs) {
    const legendSubs = this._all(".legend .sub");
    const cloudPoints = pairs.reduce((total, pair) => {
      const count = cloudPointCount(cloudFor(app, pairId(pair.id)));
      return total + (count == null ? 0 : count);
    }, 0);
    setText(legendSubs[0], `${pairs.length} pair${pairs.length === 1 ? "" : "s"} · ${cloudPoints} inliers`);
    // The second legend line is part of the static page shell.  It is never
    // rewritten by a heartbeat.
  }

  // Kept as a small compatibility seam for embedders that called the old
  // private helper while the page was still rendered as one footer block.
  _renderFooter(app, pairs) {
    this._renderFooterGauges(app);
    this._renderFooterLive(app);
    this._renderFooterLegend(app, pairs);
  }

  _renderParams(app) {
    const live = app.live || app.state || {};
    const params = live.params || live.stability_params || {};
    const serverDraft = this._normaliseParamValues(params);
    if (this._paramDraft == null) {
      this._paramDraft = serverDraft;
      this._writeParamDraft(this._paramDraft);
    } else if (!this._paramDirty) {
      if (this._paramPendingEffective != null) {
        if (this._sameParamDraft(serverDraft, this._paramPendingEffective)) {
          this._paramPendingEffective = null;
          this._paramDraft = serverDraft;
          this._writeParamDraft(this._paramDraft);
        } else {
          this._writeParamDraft(this._paramPendingEffective);
        }
      } else {
        this._paramDraft = serverDraft;
        this._writeParamDraft(this._paramDraft);
      }
    }
    const apply = this._query(".apply");
    const caution = this._query("#paramCaution");
    const writable = live.params_writable;
    if (caution) {
      const text = writable === false && live.params_detail
        ? live.params_detail
        : "Applies to future captures. Already-buffered pairs keep the gate they were taken under.";
      if (caution.textContent !== text) caution.textContent = text;
    }
    if (apply) {
      apply.disabled = writable === false || this._actionBusy;
      apply.title = writable === false
        ? (live.params_detail || "Parameter writes are disabled")
        : "Apply to future captures";
    }
    const dirty = this._paramDirty;
    this._all(".param input").forEach((input, index) => {
      const name = PARAMETER_NAMES[index];
      input.classList.toggle(
        "dirty",
        dirty && (this._paramDirtyNames.size === 0 || this._paramDirtyNames.has(name)),
      );
    });
    const reset = this._query("#paramReset");
    if (reset) {
      reset.disabled = this._actionBusy;
      reset.classList.toggle("dirty", dirty);
    }
  }

  _renderExportAvailability(app) {
    const item = this._all("#menu .item")[1];
    const box = this._query("#autoware");
    const availability = this._autowareAvailability(app);
    if (item) {
      item.disabled = !availability.available || this._actionBusy;
      item.title = availability.available
        ? "Preview the Autoware calibration diff"
        : availability.reason;
    }
    if (!box || this._autowareEntry) return;
    const text = availability.available ? "" : `Unavailable: ${availability.reason}`;
    if (text !== this._autowareAvailabilityText) {
      this._autowareAvailabilityText = text;
      setText(box, text);
    }
  }

  /** Render DOM whose revision/selection key changed. */
  render(app = {}, dirty = { all: true }) {
    this._app = app;
    this._bind();
    const state = app.captures || app.state || {};
    const live = app.live || app.state || {};
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
    const cloudKey = valueKey(cloudEntries(app));
    const listKey = valueKey({
      // ``capture_revision`` is the cheap common case; including the actual
      // projection keeps this correct for older facades and for in-place
      // updates delivered by a test/fake facade.
      captureRevision: state.capture_revision ?? null,
      capturesRevision: state.captures_revision ?? null,
      pairs,
      captures: [...captures.entries()].map(([id, capture]) => [id, capture?.position]),
      evidenceRevisions: state.evidence_revisions || {},
      clouds: cloudKey,
      selected,
    });
    // Dirty hints are scheduling hints, not truth.  Delayed evidence is
    // allowed to complete while ``list`` is false, so the value key always
    // gets the final say.
    if (listKey !== this._listRenderKey) {
      this._listRenderKey = listKey;
      this._renderList(app, pairs, captures);
    }

    const detailKey = valueKey([
      selected,
      selectedPair,
      captures.get(selected),
      app.preview,
      cloudKey,
    ]);
    if (detailKey !== this._detailRenderKey) {
      this._detailRenderKey = detailKey;
      this._renderDetail(app, selectedPair, captures.get(selected));
    }

    const footerGaugeKey = valueKey(state.diversity || {});
    if (footerGaugeKey !== this._footerGaugeRenderKey) {
      this._footerGaugeRenderKey = footerGaugeKey;
      this._renderFooterGauges(app);
    }

    const footerLiveKey = valueKey({
      stillness: live.stillness || {},
      solve: state.solve || {},
      sync: live.sync || "",
    });
    if (footerLiveKey !== this._footerLiveRenderKey) {
      this._footerLiveRenderKey = footerLiveKey;
      this._renderFooterLive(app);
    }

    const footerLegendKey = valueKey({ pairs: pairs.map((pair) => pairId(pair.id)), clouds: cloudKey });
    if (footerLegendKey !== this._footerLegendRenderKey) {
      this._footerLegendRenderKey = footerLegendKey;
      this._renderFooterLegend(app, pairs);
    }

    const paramsKey = valueKey([
      live.stability_params,
      live.params,
      live.params_writable,
      live.params_detail,
      this._actionBusy,
      this._paramDirty,
    ]);
    if (paramsKey !== this._paramsRenderKey) {
      this._paramsRenderKey = paramsKey;
      this._renderParams(app);
    }

    const exportKey = valueKey([
      live.export_availability,
      live.export,
      this._autowareEntry,
      this._actionBusy,
    ]);
    if (exportKey !== this._exportRenderKey) {
      this._exportRenderKey = exportKey;
      this._renderExportAvailability(app);
    }

    const notice = this._query("#action-notice");
    if (notice && notice.textContent !== (app.notice || "")) {
      setText(notice, app.notice || "");
    }

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
