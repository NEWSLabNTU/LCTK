/** Small, fetch-owning boundary for the assisted review server. */
export class ReviewApi {
  constructor(baseUrl = "") {
    this.baseUrl = baseUrl.replace(/\/$/, "");
    this.clouds = new Map();
    this.cloudFailures = new Map();
    this.autowareToken = null;
  }

  async _json(path, options = {}) {
    try {
      const response = await fetch(this.baseUrl + path, {
        headers: { Accept: "application/json", ...(options.headers || {}) },
        ...options,
      });
      const payload = await response.json();
      if (!response.ok) {
        return { ok: false, detail: payload.detail || response.statusText };
      }
      return payload;
    } catch (error) {
      return { ok: false, detail: error instanceof Error ? error.message : String(error) };
    }
  }

  async state() {
    return this._json("/api/state");
  }

  async scene() {
    return this._json("/api/scene");
  }

  async preview(id) {
    try {
      const response = await fetch(
        this.baseUrl + `/api/pair/${encodeURIComponent(Number(id))}/preview.jpg`,
      );
      if (!response.ok) return null;
      return await response.blob();
    } catch (_error) {
      return null;
    }
  }

  async cloud(id) {
    const key = Number(id);
    if (this.clouds.has(key)) return this.clouds.get(key);
    const failure = this.cloudFailures.get(key);
    if (failure && Date.now() < failure.nextAttemptMs) return null;
    try {
      const response = await fetch(
        this.baseUrl + `/api/pair/${encodeURIComponent(key)}/cloud.bin`,
      );
      if (!response.ok) {
        this._recordCloudFailure(key);
        return null;
      }
      const data = await response.arrayBuffer();
      this.cloudFailures.delete(key);
      this.clouds.set(key, data);
      return data;
    } catch (_error) {
      this._recordCloudFailure(key);
      return null;
    }
  }

  _recordCloudFailure(key) {
    const count = (this.cloudFailures.get(key)?.count || 0) + 1;
    const delayMs = Math.min(30000, 500 * 2 ** Math.min(count - 1, 6));
    this.cloudFailures.set(key, { count, nextAttemptMs: Date.now() + delayMs });
  }

  forgetCloud(id) {
    const key = Number(id);
    this.clouds.delete(key);
    this.cloudFailures.delete(key);
  }

  pruneClouds(activeIds) {
    const keep = new Set([...activeIds].map((id) => Number(id)));
    for (const id of this.clouds.keys()) {
      if (!keep.has(id)) this.forgetCloud(id);
    }
    for (const id of this.cloudFailures.keys()) {
      if (!keep.has(id)) this.cloudFailures.delete(id);
    }
  }

  async _post(path, body = {}) {
    return this._json(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
  }

  async drop(id) {
    const result = await this._post(`/api/pair/${encodeURIComponent(id)}/drop`);
    if (result.ok) this.forgetCloud(id);
    return result;
  }

  async exportArchive(path) {
    return this._post("/api/export/archive", { path });
  }

  async autowarePreview() {
    const result = await this._post("/api/export/autoware/preview");
    this.autowareToken = result.ok ? result.confirmation_token || null : null;
    return result;
  }

  async autowareWrite() {
    const result = await this._post("/api/export/autoware/write", {
      confirmation_token: this.autowareToken,
    });
    this.autowareToken = null;
    return result;
  }

  async setParams(values) {
    return this._post("/api/params", values);
  }
}
