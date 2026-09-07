/** Coherent client-side owner for one assisted review session. */

function numberId(value) {
  if (value == null || typeof value === "boolean") return null;
  const number = Number(value);
  return Number.isSafeInteger(number) && number >= 0 ? number : null;
}

function revisionOf(pair) {
  const value = Number(pair?.evidence_revision);
  return Number.isSafeInteger(value) && value >= 0 ? value : 0;
}

function readResult(result) {
  // The production adapter returns an envelope. Accepting a plain payload
  // keeps this seam friendly to small fakes and older embedders.
  if (result && typeof result === "object" &&
      ("payload" in result || "notModified" in result || "etag" in result)) {
    return result;
  }
  if (result && result.ok === false) return result;
  return { ok: true, notModified: false, etag: null, payload: result };
}

function pairMap(state) {
  return new Map(
    (Array.isArray(state?.pairs) ? state.pairs : [])
      .map((pair) => [numberId(pair?.id), pair])
      .filter(([id]) => id != null),
  );
}

function finiteRevision(value, fallback = null) {
  const revision = Number(value);
  return Number.isSafeInteger(revision) && revision >= 0 ? revision : fallback;
}

/**
 * ReviewSession owns cache identity and async lifetimes. The callback receives
 * a dirty-key object and the same mutable app snapshot consumed by Chrome and
 * SceneModel.
 */
export class ReviewSession {
  constructor(api, {
    onChange = () => {},
    onFocus = () => {},
    onFrameAll = () => {},
    urlApi = typeof URL === "undefined" ? null : URL,
    now = () => Date.now(),
    cloudRetryMs = 500,
    previewRetryMs = cloudRetryMs,
    previewCacheLimit = 32,
    cloudCacheLimit = 256,
    cloudConcurrency = 4,
  } = {}) {
    this.api = api;
    this.onChange = onChange;
    this.onFocus = onFocus;
    this.onFrameAll = onFrameAll;
    this.urlApi = urlApi;
    this.now = now;
    this.cloudRetryMs = Math.max(1, Number(cloudRetryMs) || 500);
    this.previewRetryMs = Math.max(1, Number(previewRetryMs) || this.cloudRetryMs);
    this.previewCacheLimit = Math.max(1, Number(previewCacheLimit) || 32);
    this.cloudCacheLimit = Math.max(1, Number(cloudCacheLimit) || 256);
    this.cloudConcurrency = Math.max(1, Number(cloudConcurrency) || 4);

    // Successful bytes live here. A missing response is a retryable failure,
    // so it is kept only in the backoff map and never in either cache.
    this.previewCache = new Map();
    this.previewPending = new Map();
    this.previewFailures = new Map();
    this.cloudCache = new Map();
    this.cloudPending = new Map();
    this.cloudFailures = new Map();
    this.cloudHydrationPromise = null;

    this.stateEtag = null;
    this.sceneEtag = null;
    this.stateRequest = null;
    this.sceneRequest = null;
    this.stateGeneration = 0;
    this.sceneGeneration = 0;
    this.assetGeneration = 0;
    this.selectionGeneration = 0;
    this.epoch = null;
    this.pairGenerations = new Map();

    this.app = {
      api,
      state: null,
      scene: { scene_revision: -1, captures: [], camera: null },
      clouds: new Map(),
      cloudRevisions: new Map(),
      selectedId: null,
      selectedCaptureId: null,
      preview: { id: null, revision: null, url: null, status: "idle" },
      notice: "",
      layers: { points: true, frustum: true, rms: true },
      sessionEpoch: null,
    };
  }

  _emit(dirty = {}) {
    this.onChange(dirty, this.app);
  }

  _activePair(id) {
    return pairMap(this.app.state).get(numberId(id)) || null;
  }

  _revokeDisplayedPreview() {
    const url = this.app.preview?.url;
    if (url && this.urlApi?.revokeObjectURL) this.urlApi.revokeObjectURL(url);
  }

  _clearAssetCaches() {
    this._revokeDisplayedPreview();
    this.previewCache.clear();
    this.previewPending.clear();
    this.previewFailures.clear();
    this.cloudCache.clear();
    this.cloudPending.clear();
    this.cloudFailures.clear();
    this.pairGenerations.clear();
    this.app.clouds.clear();
    this.app.cloudRevisions.clear();
    this.app.preview = { id: null, revision: null, url: null, status: "idle" };
  }

  _cachePreview(key, id, revision, blob) {
    this.previewCache.delete(key);
    this.previewCache.set(key, { id, revision, blob });
    while (this.previewCache.size > this.previewCacheLimit) {
      const oldest = this.previewCache.keys().next().value;
      // Keep the displayed bytes alive until the selection is detached. The
      // displayed object URL is independent of this byte cache, but retaining
      // this one entry avoids evicting the active item under a tiny test limit.
      if (oldest === this._previewKey(id, revision) && this.previewCache.size > 1) {
        const iterator = this.previewCache.keys();
        iterator.next();
        const candidate = iterator.next().value;
        if (candidate !== undefined) {
          const entry = this.previewCache.get(oldest);
          this.previewCache.delete(oldest);
          this.previewCache.set(oldest, entry);
          this.previewCache.delete(candidate);
          continue;
        }
      }
      this.previewCache.delete(oldest);
    }
  }

  _cacheCloud(key, id, revision, data) {
    this.cloudCache.delete(key);
    this.cloudCache.set(key, { id, revision, data });
    while (this.cloudCache.size > this.cloudCacheLimit) {
      const oldest = this.cloudCache.keys().next().value;
      this.cloudCache.delete(oldest);
    }
  }

  _stateEpoch(state) {
    return state?.session_epoch == null ? null : String(state.session_epoch);
  }

  _pairGeneration(id) {
    const key = numberId(id);
    return key == null ? 0 : (this.pairGenerations.get(key) || 0);
  }

  _assetNamespace() {
    return `${this.epoch == null ? "session" : this.epoch}:${this.assetGeneration}`;
  }

  _assetKey(id, revision) {
    return `${this._assetNamespace()}:${numberId(id)}:${revision}:${this._pairGeneration(id)}`;
  }

  _assetIdentity(id, revision, epoch, assetGeneration, pairGeneration) {
    const current = this._activePair(id);
    return this.epoch === epoch &&
      this.assetGeneration === assetGeneration &&
      this._pairGeneration(id) === pairGeneration &&
      Boolean(current) && revisionOf(current) === revision;
  }

  _previewKey(id, evidenceRevision) {
    return this._assetKey(id, evidenceRevision);
  }

  _touchCache(map, key) {
    const value = map.get(key);
    if (value !== undefined) {
      map.delete(key);
      map.set(key, value);
    }
    return value;
  }

  _setPreviewLoading(id, evidenceRevision = revisionOf(this._activePair(id))) {
    this._revokeDisplayedPreview();
    this.app.preview = { id, revision: evidenceRevision, url: null, status: "loading" };
    this._emit({ detail: true, preview: true, selection: true });
  }

  _setPreviewMissingIfCurrent(id, evidenceRevision, generation) {
    if (generation !== this.selectionGeneration || this.app.selectedId !== id) return;
    this.app.preview = { id, revision: evidenceRevision, url: null, status: "missing" };
    this._emit({ detail: true, preview: true });
  }

  _attachPreviewIfCurrent(blob, id, evidenceRevision, generation) {
    if (generation !== this.selectionGeneration || this.app.selectedId !== id) return false;
    const current = this._activePair(id);
    if (!current || revisionOf(current) !== evidenceRevision) return false;
    if (blob == null) {
      this._setPreviewMissingIfCurrent(id, evidenceRevision, generation);
      return false;
    }
    this._revokeDisplayedPreview();
    const url = this.urlApi?.createObjectURL ? this.urlApi.createObjectURL(blob) : null;
    this.app.preview = {
      id,
      revision: evidenceRevision,
      url,
      status: url ? "ready" : "missing",
    };
    this._emit({ detail: true, preview: true });
    return Boolean(url);
  }

  _previewBackoff(key) {
    const failure = this.previewFailures.get(key);
    return failure && this.now() < failure.nextAttemptMs ? failure : null;
  }

  _cloudBackoff(key) {
    const failure = this.cloudFailures.get(key);
    return failure && this.now() < failure.nextAttemptMs ? failure : null;
  }

  _recordFailure(map, key, baseDelayMs, previous) {
    const count = (previous?.count || 0) + 1;
    map.set(key, {
      count,
      nextAttemptMs: this.now() + Math.min(30000, baseDelayMs * 2 ** Math.min(count - 1, 6)),
    });
  }

  async refreshState() {
    if (this.stateRequest) return this.stateRequest;
    const requestGeneration = this.stateGeneration;
    const request = (async () => {
      let raw;
      try {
        raw = await this.api.state(this.stateEtag);
      } catch (error) {
        if (requestGeneration !== this.stateGeneration) return false;
        this.app.notice = `server unavailable: ${error instanceof Error ? error.message : String(error)}`;
        this._emit({ notice: true, live: true });
        return false;
      }
      if (requestGeneration !== this.stateGeneration) return false;
      const result = readResult(raw);
      if (result.ok === false) {
        this.app.notice = `server unavailable: ${result.detail || "request failed"}`;
        this._emit({ notice: true, live: true });
        return false;
      }
      if (result.notModified) {
        // Live status may be unchanged while an asset request failed. Keep the
        // retained state and run retryable hydration on every heartbeat.
        if (result.etag) this.stateEtag = result.etag;
        this._scheduleHydration();
        return false;
      }
      const state = result.payload;
      if (!state || state.ok === false) {
        this.app.notice = `server unavailable: ${state?.detail || "request failed"}`;
        this._emit({ notice: true, live: true });
        return false;
      }
      this.stateEtag = result.etag || this.stateEtag;
      const oldState = this.app.state;
      const oldEpoch = this.epoch;
      const nextEpoch = this._stateEpoch(state);
      const epochChanged = oldEpoch != null && nextEpoch != null && oldEpoch !== nextEpoch;
      if (epochChanged) {
        this.assetGeneration += 1;
        this.sceneGeneration += 1;
        this.sceneRequest = null;
        this._clearAssetCaches();
        this.sceneEtag = null;
        this.app.scene = { scene_revision: -1, captures: [], camera: null };
        this.app.selectedId = null;
        this.app.selectedCaptureId = null;
        this.selectionGeneration += 1;
        this.onFrameAll();
      }
      if (nextEpoch != null) this.epoch = nextEpoch;
      this.app.sessionEpoch = this.epoch;

      const oldPairs = pairMap(oldState);
      const nextPairs = pairMap(state);
      for (const id of oldPairs.keys()) {
        if (!nextPairs.has(id)) this._forgetPair(id);
      }
      const oldCaptureRevision = finiteRevision(oldState?.capture_revision);
      const nextCaptureRevision = finiteRevision(state.capture_revision);
      const captureChanged = oldState == null || oldCaptureRevision !== nextCaptureRevision;
      const oldSceneRevision = finiteRevision(oldState?.scene_revision);
      const nextSceneRevision = finiteRevision(state.scene_revision);
      const sceneChanged = oldState == null || oldSceneRevision !== nextSceneRevision;
      if (sceneChanged && !epochChanged) {
        // A request for the previous scene must not be allowed to satisfy the
        // new state. The old promise may still finish, but its generation check
        // makes its result inert.
        this.sceneGeneration += 1;
        this.sceneRequest = null;
      }
      // From this point onward all selection and asset guards must inspect the
      // newest accepted state. Keeping the old state here makes an evidence
      // revision change invisible to the selected preview/cloud paths.
      this.app.state = state;
      const selected = numberId(this.app.selectedId);
      let selectionDropped = false;
      if (selected != null && !nextPairs.has(selected)) {
        this._clearSelectionState();
        selectionDropped = true;
      }
      const selectedPair = this._activePair(selected);
      if (selectedPair && this.app.preview.id === selected &&
          this.app.preview.revision !== revisionOf(selectedPair)) {
        this._revokeDisplayedPreview();
        this.app.preview = {
          id: selected,
          revision: revisionOf(selectedPair),
          url: null,
          status: selectedPair.has_preview === true ? "loading" : "missing",
        };
      }
      this._pruneClouds();
      // This is the first useful paint. Hydration is deliberately scheduled only
      // after this callback, never awaited by the state request.
      this._emit({
        state: true,
        list: captureChanged || selectionDropped,
        footer: captureChanged || oldState == null,
        live: true,
        scene: sceneChanged,
        detail: selectionDropped,
        preview: selectionDropped,
      });
      if (selectionDropped) this.onFrameAll();
      this._scheduleHydration();
      return true;
    })();
    this.stateRequest = request;
    try {
      return await request;
    } finally {
      if (this.stateRequest === request) this.stateRequest = null;
    }
  }

  async refreshScene() {
    const state = this.app.state;
    if (!state) return null;
    if (this.sceneRequest) return this.sceneRequest;
    const expectedEpoch = this.epoch;
    const expectedStateGeneration = this.stateGeneration;
    const expectedRevision = finiteRevision(state.scene_revision, -1);
    if (finiteRevision(this.app.scene?.scene_revision, -2) === expectedRevision) return null;
    const requestGeneration = this.sceneGeneration;
    const request = (async () => {
      let raw;
      try {
        raw = await this.api.scene(this.sceneEtag);
      } catch (_error) {
        return false;
      }
      if (requestGeneration !== this.sceneGeneration ||
          expectedStateGeneration !== this.stateGeneration ||
          expectedEpoch !== this.epoch) return false;
      const result = readResult(raw);
      if (result.ok === false) return false;
      if (result.notModified) {
        // A 304 is useful only when the retained scene already represents the
        // current state revision. Otherwise the validator was stale and the
        // next heartbeat must retry without accepting a mismatched scene.
        if (finiteRevision(this.app.scene?.scene_revision, -2) !== expectedRevision) {
          this.sceneEtag = null;
          return false;
        }
        if (result.etag) this.sceneEtag = result.etag;
        return false;
      }
      const scene = result.payload;
      if (!scene || scene.ok === false) return false;
      const currentState = this.app.state;
      const sceneEpoch = this._stateEpoch(scene);
      if (expectedEpoch !== this.epoch ||
          expectedStateGeneration !== this.stateGeneration ||
          !currentState ||
          finiteRevision(currentState.scene_revision, -1) !== finiteRevision(scene.scene_revision, -2) ||
          finiteRevision(scene.scene_revision, -2) !== expectedRevision ||
          (sceneEpoch != null && sceneEpoch !== this.epoch)) return false;
      this.sceneEtag = result.etag || this.sceneEtag;
      this.app.scene = scene;
      this._emit({ scene: true });
      this._scheduleHydration();
      return true;
    })();
    this.sceneRequest = request;
    try {
      return await request;
    } finally {
      if (this.sceneRequest === request) this.sceneRequest = null;
    }
  }

  async _loadCloud(id, evidenceRevision) {
    const key = this._assetKey(id, evidenceRevision);
    const expectedEpoch = this.epoch;
    const expectedAssetGeneration = this.assetGeneration;
    const expectedPairGeneration = this._pairGeneration(id);
    const currentCloudRevision = this.app.cloudRevisions.get(id);
    if (currentCloudRevision !== evidenceRevision && this.app.clouds.has(id)) {
      this.app.clouds.delete(id);
      this.app.cloudRevisions.delete(id);
      this._emit({ scene: true, cloud: [id] });
    }
    const cached = this._touchCache(this.cloudCache, key);
    if (cached) {
      if (this._assetIdentity(id, evidenceRevision, expectedEpoch,
          expectedAssetGeneration, expectedPairGeneration) &&
          this.app.clouds.get(id) !== cached.data) {
        this.app.clouds.set(id, cached.data);
        this.app.cloudRevisions.set(id, evidenceRevision);
        this._emit({ scene: true, cloud: [id] });
      }
      return cached.data;
    }
    if (this._cloudBackoff(key)) return null;
    if (this.cloudPending.has(key)) return this.cloudPending.get(key);
    const promise = (async () => {
      let data;
      try {
        data = await this.api.cloud(id);
      } catch (_error) {
        data = null;
      }
      if (data == null) {
        if (this._assetIdentity(id, evidenceRevision, expectedEpoch,
            expectedAssetGeneration, expectedPairGeneration)) {
          this._recordFailure(this.cloudFailures, key, this.cloudRetryMs,
            this.cloudFailures.get(key));
        }
        return null;
      }
      // A response from an old node epoch or dropped pair must not repopulate
      // either the current UI or its byte cache.
      if (!this._assetIdentity(id, evidenceRevision, expectedEpoch,
          expectedAssetGeneration, expectedPairGeneration)) return data;
      this.cloudFailures.delete(key);
      this._cacheCloud(key, id, evidenceRevision, data);
      if (this.app.clouds.get(id) !== data) {
        this.app.clouds.set(id, data);
        this.app.cloudRevisions.set(id, evidenceRevision);
        this._emit({ scene: true, cloud: [id] });
      }
      return data;
    })();
    this.cloudPending.set(key, promise);
    try {
      return await promise;
    } finally {
      if (this.cloudPending.get(key) === promise) this.cloudPending.delete(key);
    }
  }

  async _hydrateCloudsOnce() {
    const captures = (this.app.scene?.captures || []).map((capture) => {
      const id = numberId(capture?.id);
      const pair = this._activePair(id);
      if (id == null || !pair) return null;
      const revision = revisionOf(pair);
      const key = this._assetKey(id, revision);
      if (this.app.cloudRevisions.get(id) === revision && this.app.clouds.has(id)) return null;
      if (this.cloudCache.has(key) || this.cloudPending.has(key) || this._cloudBackoff(key)) return null;
      return { id, revision };
    }).filter(Boolean);
    if (!captures.length) return [];

    let next = 0;
    const worker = async () => {
      while (next < captures.length) {
        const index = next++;
        const task = captures[index];
        try {
          await this._loadCloud(task.id, task.revision);
        } catch (_error) {
          // One malformed response must not prevent the remaining captures
          // from hydrating on this pass.
        }
      }
    };
    const workers = Array.from(
      { length: Math.min(this.cloudConcurrency, captures.length) },
      () => worker(),
    );
    return Promise.all(workers);
  }

  async hydrateClouds() {
    // Heartbeats can arrive while a previous batch is still waiting on a
    // slow cloud endpoint. Keep one scheduler promise so the configured
    // worker limit is global, not multiplied by each heartbeat.
    if (this.cloudHydrationPromise) return this.cloudHydrationPromise;
    const promise = this._hydrateCloudsOnce();
    this.cloudHydrationPromise = promise;
    try {
      return await promise;
    } finally {
      if (this.cloudHydrationPromise === promise) this.cloudHydrationPromise = null;
    }
  }

  async _loadPreview(id, evidenceRevision, generation) {
    const key = this._previewKey(id, evidenceRevision);
    const cached = this._touchCache(this.previewCache, key);
    if (cached) {
      this._attachPreviewIfCurrent(cached.blob, id, evidenceRevision, generation);
      return cached.blob;
    }
    const existing = this.previewPending.get(key);
    if (existing) {
      // A second selection of A while A's first request is pending must attach
      // the shared result using the newer selection generation.
      const blob = await existing;
      this._attachPreviewIfCurrent(blob, id, evidenceRevision, generation);
      return blob;
    }
    const failure = this._previewBackoff(key);
    if (failure) {
      this._setPreviewMissingIfCurrent(id, evidenceRevision, generation);
      return null;
    }
    const expectedEpoch = this.epoch;
    const expectedAssetGeneration = this.assetGeneration;
    const expectedPairGeneration = this._pairGeneration(id);
    const promise = (async () => {
      let blob;
      try {
        blob = await this.api.preview(id);
      } catch (_error) {
        blob = null;
      }
      if (blob == null) {
        if (this._assetIdentity(id, evidenceRevision, expectedEpoch,
            expectedAssetGeneration, expectedPairGeneration)) {
          this._recordFailure(this.previewFailures, key, this.previewRetryMs,
            this.previewFailures.get(key));
        }
        return null;
      }
      // Guard before caching as well as before attaching. Bytes from a prior
      // epoch or dropped pair are not reusable by the current session.
      if (!this._assetIdentity(id, evidenceRevision, expectedEpoch,
          expectedAssetGeneration, expectedPairGeneration)) return blob;
      this.previewFailures.delete(key);
      this._cachePreview(key, id, evidenceRevision, blob);
      return blob;
    })();
    this.previewPending.set(key, promise);
    try {
      const blob = await promise;
      this._attachPreviewIfCurrent(blob, id, evidenceRevision, generation);
      return blob;
    } finally {
      if (this.previewPending.get(key) === promise) this.previewPending.delete(key);
    }
  }

  select(value) {
    const id = numberId(value);
    if (id == null || !this._activePair(id)) return this.clearSelection();
    this.selectionGeneration += 1;
    const generation = this.selectionGeneration;
    this.app.selectedId = id;
    this.app.selectedCaptureId = id;
    this.onFocus(id);
    const pair = this._activePair(id);
    const evidenceRevision = revisionOf(pair);
    const hasPreview = pair?.has_preview === true;
    const key = this._previewKey(id, evidenceRevision);
    const cached = this.previewCache.has(key);
    if (!hasPreview && !cached) {
      this._revokeDisplayedPreview();
      this.app.preview = { id, revision: evidenceRevision, url: null, status: "missing" };
      this._emit({ selection: true, detail: true, preview: true });
      return;
    }
    this._setPreviewLoading(id, evidenceRevision);
    void this._loadPreview(id, evidenceRevision, generation);
    // `_setPreviewLoading` already emits selection/detail/preview. Keep this
    // second event small for callers that inspect selection independently.
    this._emit({ selection: true, detail: true });
  }

  _clearSelectionState() {
    this.selectionGeneration += 1;
    this._revokeDisplayedPreview();
    this.app.selectedId = null;
    this.app.selectedCaptureId = null;
    this.app.preview = { id: null, revision: null, url: null, status: "idle" };
  }

  clearSelection() {
    this._clearSelectionState();
    this._emit({ selection: true, detail: true, preview: true });
    this.onFrameAll();
  }

  _pruneClouds() {
    const active = pairMap(this.app.state);
    for (const id of this.app.clouds.keys()) {
      const pair = active.get(id);
      if (!pair || this.app.cloudRevisions.get(id) !== revisionOf(pair)) {
        this.app.clouds.delete(id);
        this.app.cloudRevisions.delete(id);
      }
    }
    for (const [key, entry] of this.cloudCache) {
      if (!active.has(entry.id)) this.cloudCache.delete(key);
    }
    this._prunePreviewCaches(active);
  }

  _prunePreviewCaches(active) {
    for (const [key, entry] of this.previewCache) {
      if (!active.has(entry.id)) this.previewCache.delete(key);
    }
  }

  _forgetPair(id) {
    const keyId = numberId(id);
    if (keyId == null) return;
    this.pairGenerations.set(keyId, this._pairGeneration(keyId) + 1);
    for (const [key, entry] of this.previewCache) {
      if (entry.id === keyId) this.previewCache.delete(key);
    }
    for (const [key, entry] of this.cloudCache) {
      if (entry.id === keyId) this.cloudCache.delete(key);
    }
    for (const key of this.previewFailures.keys()) {
      if (key.includes(`:${keyId}:`)) this.previewFailures.delete(key);
    }
    for (const key of this.cloudFailures.keys()) {
      if (key.includes(`:${keyId}:`)) this.cloudFailures.delete(key);
    }
    this.app.clouds.delete(keyId);
    this.app.cloudRevisions.delete(keyId);
  }

  _scheduleHydration() {
    void this.refreshScene();
    void this.hydrateClouds();
    const id = numberId(this.app.selectedId);
    const pair = this._activePair(id);
    const previewKey = id == null || !pair
      ? null
      : this._previewKey(id, revisionOf(pair));
    if (id == null || !pair || pair.has_preview !== true || this.app.preview.status === "ready") return;
    if (this.previewPending.has(previewKey)) return;
    const failure = this._previewBackoff(previewKey);
    if (failure) {
      this._setPreviewMissingIfCurrent(id, revisionOf(pair), this.selectionGeneration);
      return;
    }
    this._setPreviewLoading(id, revisionOf(pair));
    void this._loadPreview(id, revisionOf(pair), this.selectionGeneration);
  }

  async start() {
    await this.refreshState();
  }

  async poll() {
    await this.refreshState();
  }

  async drop(id) {
    let result;
    try {
      result = await this.api.drop(id);
    } catch (error) {
      return { ok: false, detail: error instanceof Error ? error.message : String(error) };
    }
    if (result?.ok) {
      this._forgetPair(id);
      if (this.app.selectedId === numberId(id)) {
        this.clearSelection();
      } else {
        this._emit({ state: true, list: true, footer: true, scene: true });
      }
    }
    return result;
  }

  reset() {
    this.stateGeneration += 1;
    this.sceneGeneration += 1;
    this.assetGeneration += 1;
    this.selectionGeneration += 1;
    this.stateRequest = null;
    this.sceneRequest = null;
    this._clearAssetCaches();
    this.stateEtag = null;
    this.sceneEtag = null;
    this.epoch = null;
    this.app.state = null;
    this.app.scene = { scene_revision: -1, captures: [], camera: null };
    this.app.selectedId = null;
    this.app.selectedCaptureId = null;
    this.app.sessionEpoch = null;
    this.app.notice = "";
    this._emit({ state: true, scene: true, list: true, footer: true, detail: true, preview: true });
  }
}

export default ReviewSession;
