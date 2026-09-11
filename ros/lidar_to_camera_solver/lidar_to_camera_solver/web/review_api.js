/** HTTP-only boundary for the assisted review server.
 *
 * ReviewSession owns state, validators, retries, and asset caches. Keeping
 * this module transport-only prevents a second cache from diverging from it.
 */
export class ReviewApi {
  constructor(baseUrl = "") {
    this.baseUrl = baseUrl.replace(/\/$/, "");
    this.autowareToken = null;
  }

  async _json(path, options = {}) {
    try {
      const response = await fetch(this.baseUrl + path, {
        headers: { Accept: "application/json", ...(options.headers || {}) },
        ...options,
      });
      let payload = null;
      try {
        payload = await response.json();
      } catch (_error) {
        payload = {};
      }
      if (!response.ok) {
        return { ok: false, detail: payload?.detail || response.statusText };
      }
      return payload;
    } catch (error) {
      return { ok: false, detail: error instanceof Error ? error.message : String(error) };
    }
  }

  async _read(path, options = {}) {
    try {
      const response = await fetch(this.baseUrl + path, {
        headers: { Accept: "application/json", ...(options.headers || {}) },
        ...options,
      });
      const etag = response.headers?.get?.("ETag") || null;
      if (response.status === 304) return { ok: true, notModified: true, etag };
      let payload = null;
      try {
        payload = await response.json();
      } catch (_error) {
        payload = {};
      }
      if (!response.ok) {
        return { ok: false, detail: payload?.detail || response.statusText };
      }
      return { ok: true, notModified: false, etag, payload };
    } catch (error) {
      return { ok: false, detail: error instanceof Error ? error.message : String(error) };
    }
  }

  async state(etag = null) {
    return this._read("/api/state", {
      headers: etag ? { "If-None-Match": etag } : {},
    });
  }

  async live(etag = null) {
    return this._read("/api/live", {
      headers: etag ? { "If-None-Match": etag } : {},
    });
  }

  async captures(etag = null) {
    return this._read("/api/captures", {
      headers: etag ? { "If-None-Match": etag } : {},
    });
  }

  async revisions(etag = null) {
    return this._read("/api/revisions", {
      headers: etag ? { "If-None-Match": etag } : {},
    });
  }

  eventsUrl() {
    return this.baseUrl + "/api/events";
  }

  openEvents() {
    if (typeof EventSource !== "function") return null;
    return new EventSource(this.eventsUrl());
  }

  async scene(etag = null) {
    return this._read("/api/scene", {
      headers: etag ? { "If-None-Match": etag } : {},
    });
  }

  async preview(id) {
    try {
      const response = await fetch(
        this.baseUrl + `/api/pair/${encodeURIComponent(Number(id))}/preview.jpg`,
        { headers: { Accept: "image/jpeg" } },
      );
      if (!response.ok) return null;
      return await response.blob();
    } catch (_error) {
      return null;
    }
  }

  async cloud(id) {
    try {
      const response = await fetch(
        this.baseUrl + `/api/pair/${encodeURIComponent(Number(id))}/cloud.bin`,
        { headers: { Accept: "application/octet-stream" } },
      );
      if (!response.ok) return null;
      return await response.arrayBuffer();
    } catch (_error) {
      return null;
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
    return this._post(`/api/pair/${encodeURIComponent(id)}/drop`);
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

export default ReviewApi;
