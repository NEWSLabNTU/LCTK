"""The capture-review page and its JSON API.

Everything here talks to the node through :class:`NodeFacade`, never to ROS
types, so the whole module is testable with Flask's test client and a fake. The
server runs on its own thread; the facade is what makes that safe, because the
node implements it with the correct locking on the other side.

The Autoware export is deliberately two requests. It writes a file that reaches a
vehicle, so the operator sees the diff first and confirms second, and a buffer
change between the two invalidates the confirmation.
"""

from __future__ import annotations

import threading
from typing import Any, Protocol

from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server

_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LCTK assisted capture</title>
<style>
 :root { color-scheme: light dark; }
 body { font: 14px/1.45 system-ui, sans-serif; margin: 0; padding: 1rem; }
 h1 { font-size: 1.1rem; margin: 0 0 .75rem; }
 .banner { padding: .6rem .8rem; border-radius: .4rem; margin-bottom: .8rem;
           background: #7773; }
 .banner.still { background: #2b8a3e33; }
 .shortfall { margin: .15rem 0; opacity: .85; }
 .pair { display: flex; gap: .8rem; align-items: center;
         border-top: 1px solid #8884; padding: .5rem 0; }
 .pair img { width: 220px; border-radius: .25rem; background: #8882; }
 .worst { outline: 2px solid #e0348044; }
 button { font: inherit; padding: .35rem .7rem; border-radius: .3rem; }
 .sync { opacity: .7; font-size: .85em; }
 pre { background: #8882; padding: .5rem; border-radius: .3rem; overflow-x: auto; }
</style>
</head>
<body>
<h1>LCTK assisted capture <span class="sync" id="sync"></span></h1>
<div class="banner" id="banner">connecting…</div>
<div id="diversity"></div>
<div id="solve"></div>
<div id="pairs"></div>
<p>
  <button onclick="exportArchive()">Export archive</button>
  <button onclick="autowarePreview()">Export to Autoware…</button>
</p>
<div id="autoware"></div>
<script>
let archivePath = "";
async function refresh() {
  const state = await (await fetch("/api/state")).json();
  archivePath = state.export.archive_path || "";
  document.getElementById("sync").textContent = state.sync || "";
  const banner = document.getElementById("banner");
  banner.textContent = state.stillness.reason || "";
  banner.className = "banner" + (state.stillness.is_still ? " still" : "");
  document.getElementById("diversity").innerHTML =
    (state.diversity.shortfalls || [])
      .map(s => '<div class="shortfall">· ' + s + "</div>").join("")
    || '<div class="shortfall">diversity targets met</div>';
  const solve = state.solve || {};
  document.getElementById("solve").textContent =
    "solve: " + (solve.status || "?") +
    (solve.rms_px != null ? "  RMS " + solve.rms_px.toFixed(2) + " px" : "") +
    (solve.detail ? "  " + solve.detail : "");
  const pairs = (state.pairs || []).slice().sort(
    (a, b) => (b.rms_px || 0) - (a.rms_px || 0));
  document.getElementById("pairs").innerHTML = pairs.map((p, i) =>
    '<div class="pair' + (i === 0 && pairs.length > 1 ? ' worst' : '') + '">' +
    (p.has_preview
      ? '<img src="/api/pair/' + p.id + '/preview.jpg?v=' + p.id + '">'
      : '<img alt="no frame">') +
    '<div>#' + p.id +
    (p.rms_px != null ? '<br>' + p.rms_px.toFixed(2) + ' px' : '') + '</div>' +
    '<button onclick="dropPair(' + p.id + ')">drop</button></div>').join("");
}
async function post(url, body) {
  const response = await fetch(url, {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body || {}),
  });
  return response.json();
}
async function dropPair(id) { await post("/api/pair/" + id + "/drop"); refresh(); }
async function exportArchive() {
  const result = await post("/api/export/archive", {path: archivePath});
  alert(result.detail);
}
async function autowarePreview() {
  const result = await post("/api/export/autoware/preview");
  const box = document.getElementById("autoware");
  if (!result.ok) { box.textContent = result.detail; return; }
  box.innerHTML = "<pre>" + JSON.stringify(result.entry, null, 2) + "</pre>" +
    '<button onclick="autowareWrite()">Confirm write</button>';
}
async function autowareWrite() {
  const result = await post("/api/export/autoware/write");
  document.getElementById("autoware").textContent = result.detail;
}
setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
"""


class NodeFacade(Protocol):
    """What the server needs from the node. Plain data in, plain data out.

    No method raises: failures come back as ``(False, reason)`` so an operator
    sees the reason on the page instead of a stack trace in a log.
    """

    def state(self) -> dict[str, Any]: ...

    def preview(self, pair_id: int) -> bytes | None: ...

    def drop(self, pair_id: int) -> tuple[bool, str]: ...

    def export_archive(self, path: str) -> tuple[bool, str]: ...

    def export_autoware(self, dry_run: bool) -> tuple[bool, str, dict | None]: ...


def create_app(facade: NodeFacade) -> Flask:
    app = Flask(__name__)
    # The confirmation token for the Autoware write: the buffer revision the
    # operator was shown a diff for. Any mutation clears it.
    pending: dict[str, Any] = {"previewed": False}

    @app.get("/")
    def index() -> Response:
        return Response(_PAGE, mimetype="text/html")

    @app.get("/api/state")
    def state() -> Response:
        return jsonify(facade.state())

    @app.get("/api/pair/<int:pair_id>/preview.jpg")
    def preview(pair_id: int) -> Response:
        data = facade.preview(pair_id)
        if data is None:
            return Response("no preview for that pair", status=404)
        return Response(data, mimetype="image/jpeg")

    @app.post("/api/pair/<int:pair_id>/drop")
    def drop(pair_id: int) -> Response:
        ok, detail = facade.drop(pair_id)
        if ok:
            # The diff the operator was shown described a different buffer.
            pending["previewed"] = False
        return jsonify({"ok": ok, "detail": detail})

    @app.post("/api/export/archive")
    def export_archive() -> Response:
        payload = request.get_json(silent=True) or {}
        path = payload.get("path")
        if not path:
            return jsonify({"ok": False, "detail": "no 'path' given for the archive"})
        ok, detail = facade.export_archive(path)
        return jsonify({"ok": ok, "detail": detail})

    @app.post("/api/export/autoware/preview")
    def autoware_preview() -> Response:
        ok, detail, entry = facade.export_autoware(dry_run=True)
        pending["previewed"] = ok
        return jsonify({"ok": ok, "detail": detail, "entry": entry})

    @app.post("/api/export/autoware/write")
    def autoware_write() -> Response:
        if not pending["previewed"]:
            return jsonify(
                {
                    "ok": False,
                    "detail": "preview the Autoware diff first; nothing is written "
                    "unseen, and a buffer change invalidates an earlier preview",
                    "entry": None,
                }
            )
        ok, detail, entry = facade.export_autoware(dry_run=False)
        pending["previewed"] = False
        return jsonify({"ok": ok, "detail": detail, "entry": entry})

    return app


class ReviewServer:
    """The Flask app on a daemon thread, startable and stoppable by the node."""

    def __init__(self, facade: NodeFacade, *, host: str, port: int):
        self._server = make_server(host, port, create_app(facade), threaded=True)
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="lctk-review", daemon=True
        )

    @property
    def port(self) -> int:
        return self._server.server_port

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._server.shutdown()
        self._thread.join(timeout=2.0)
