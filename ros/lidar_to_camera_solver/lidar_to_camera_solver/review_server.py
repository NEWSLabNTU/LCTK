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

import ipaddress
import math
import threading
from pathlib import Path
from typing import Any, Protocol

from flask import Flask, Response, jsonify, request
from werkzeug.serving import make_server

STABILITY_PARAMETER_NAMES = frozenset(
    {
        "stability_window_s",
        "stability_max_translation_m",
        "stability_max_rotation_deg",
    }
)


def validate_stability_params(values: object) -> str | None:
    """Return an operator-facing validation error, or ``None`` when valid."""

    if not isinstance(values, dict) or not values:
        return "request must be a non-empty JSON object"
    unknown = sorted(
        (name for name in values if name not in STABILITY_PARAMETER_NAMES),
        key=str,
    )
    if unknown:
        return f"parameter '{unknown[0]}' is not settable"
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return f"parameter '{name}' must be a finite number"
        try:
            numeric = float(value)
        except (OverflowError, ValueError, TypeError):
            return f"parameter '{name}' must be finite and strictly positive"
        if not math.isfinite(numeric) or numeric <= 0.0:
            return f"parameter '{name}' must be finite and strictly positive"
    return None


class NodeFacade(Protocol):
    """What the server needs from the node. Plain data in, plain data out.

    No method raises: failures come back as ``(False, reason)`` so an operator
    sees the reason on the page instead of a stack trace in a log.
    """

    def state(self) -> dict[str, Any]: ...

    def scene(self) -> dict[str, Any]: ...

    def set_stability_params(self, values: dict[str, Any]) -> tuple[bool, str]: ...

    def preview(self, pair_id: int) -> bytes | None: ...

    def cloud(self, pair_id: int) -> bytes | None: ...

    def drop(self, pair_id: int) -> tuple[bool, str]: ...

    def export_archive(self, path: str) -> tuple[bool, str]: ...

    def export_autoware(self, dry_run: bool) -> tuple[bool, str, dict | None]: ...


def is_loopback_host(host: str) -> bool:
    """Return whether ``host`` is a numeric loopback address."""

    try:
        return ipaddress.ip_address(host).is_loopback
    except (TypeError, ValueError):
        return False


def create_app(facade: NodeFacade, *, params_writable: bool = True) -> Flask:
    static_folder = Path(__file__).resolve().parent / "web"
    app = Flask(
        __name__,
        static_folder=str(static_folder),
        static_url_path="/static",
    )
    # The confirmation token for the Autoware write: the buffer revision the
    # operator was shown a diff for. Any mutation clears it.
    pending: dict[str, Any] = {"previewed": False}

    @app.get("/")
    def index() -> Response:
        return app.send_static_file("index.html")

    @app.get("/api/state")
    def state() -> Response:
        return jsonify(facade.state())

    @app.get("/api/scene")
    def scene() -> Response:
        return jsonify(facade.scene())

    @app.post("/api/params")
    def set_params() -> Response:
        if not params_writable:
            return (
                jsonify(
                    {
                        "ok": False,
                        "detail": (
                            "parameter writes are disabled: the review server is "
                            "not bound to a loopback address"
                        ),
                    }
                ),
                403,
            )
        payload = request.get_json(silent=True)
        validation_error = validate_stability_params(payload)
        if validation_error is not None:
            return (
                jsonify(
                    {
                        "ok": False,
                        "detail": validation_error,
                    }
                ),
                400,
            )
        ok, detail = facade.set_stability_params(payload)
        if not ok:
            return jsonify({"ok": False, "detail": detail}), 400
        effective = {}
        try:
            state = facade.state()
            effective = state.get("stability_params", {})
        except (AttributeError, TypeError):
            effective = {}
        return jsonify({"ok": True, "detail": detail, "params": effective})

    @app.get("/api/pair/<int:pair_id>/preview.jpg")
    def preview(pair_id: int) -> Response:
        data = facade.preview(pair_id)
        if data is None:
            return Response("no preview for that pair", status=404)
        return Response(data, mimetype="image/jpeg")

    @app.get("/api/pair/<int:pair_id>/cloud.bin")
    def cloud(pair_id: int) -> Response:
        data = facade.cloud(pair_id)
        if data is None:
            return Response("no plane inliers for that pair", status=404)
        return Response(data, mimetype="application/octet-stream")

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
        self._server = make_server(
            host,
            port,
            create_app(facade, params_writable=is_loopback_host(host)),
            threaded=True,
        )
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
