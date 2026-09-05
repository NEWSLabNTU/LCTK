/** Small, fetch-owning boundary for the assisted review server. */
export class ReviewApi {
  constructor(baseUrl = "") {
    this.baseUrl = baseUrl.replace(/\/$/, "");
    this.clouds = new Map();
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
    try {
      const response = await fetch(
        this.baseUrl + `/api/pair/${encodeURIComponent(key)}/cloud.bin`,
      );
      if (!response.ok) {
        this.clouds.set(key, null);
        return null;
      }
      const data = await response.arrayBuffer();
      this.clouds.set(key, data);
      return data;
    } catch (_error) {
      return null;
    }
  }

  forgetCloud(id) {
    this.clouds.delete(Number(id));
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
    return this._post("/api/export/autoware/preview");
  }

  async autowareWrite() {
    return this._post("/api/export/autoware/write");
  }

  async setParams(values) {
    return this._post("/api/params", values);
  }
}
