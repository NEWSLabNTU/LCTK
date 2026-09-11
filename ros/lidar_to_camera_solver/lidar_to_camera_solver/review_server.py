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

import hashlib
import ipaddress
import json
import math
import secrets
import threading
from collections.abc import Callable, Mapping
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

    def live(self) -> dict[str, Any]: ...

    def captures(self) -> dict[str, Any]: ...

    def revisions(self) -> dict[str, Any]: ...

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


_REVISION_KEYS = (
    "session_epoch",
    "live_revision",
    "captures_revision",
    "scene_revision",
)
_DEFAULT_REVISION_VECTOR: dict[str, int | str] = {
    "session_epoch": "",
    "live_revision": 0,
    "captures_revision": 0,
    "scene_revision": 0,
}


def _revision_vector(value: object) -> dict[str, int | str] | None:
    """Normalize a read-model snapshot to the compact public event vector."""

    if hasattr(value, "event_vector"):
        try:
            value = value.event_vector()
        except (AttributeError, TypeError, ValueError):
            return None
    if not isinstance(value, Mapping):
        return None
    epoch = value.get("session_epoch", "")
    if epoch is None:
        epoch = ""
    raw_revisions = {
        "live_revision": value.get("live_revision", value.get("state_revision", 0)),
        "captures_revision": value.get(
            "captures_revision", value.get("capture_revision", 0)
        ),
        "scene_revision": value.get("scene_revision", 0),
    }
    if any(isinstance(raw, bool) for raw in raw_revisions.values()):
        return None
    try:
        revisions = {
            "session_epoch": str(epoch),
            "live_revision": int(raw_revisions["live_revision"]),
            "captures_revision": int(raw_revisions["captures_revision"]),
            "scene_revision": int(raw_revisions["scene_revision"]),
        }
    except (TypeError, ValueError, OverflowError):
        return None
    if any(revisions[key] < 0 for key in _REVISION_KEYS[1:]):
        return None
    return revisions


def _mapping_payload(value: object) -> dict[str, Any]:
    """Copy a facade representation without allowing it to mutate in Flask."""

    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _state_projection(state: object, fields: tuple[str, ...]) -> dict[str, Any]:
    """Build a compatibility split representation from the old full state."""

    if not isinstance(state, Mapping):
        return {}
    return {field: state[field] for field in fields if field in state}


def _read_representation(
    facade: NodeFacade,
    name: str,
    fallback_fields: tuple[str, ...],
) -> dict[str, Any]:
    """Read a split representation, retaining compatibility with old facades."""

    reader = getattr(facade, name, None)
    if callable(reader):
        try:
            payload = _mapping_payload(reader())
        except (AttributeError, KeyError, TypeError, ValueError):
            payload = {}
        if payload:
            return payload
    try:
        return _state_projection(facade.state(), fallback_fields)
    except (AttributeError, KeyError, TypeError, ValueError):
        return {}


def _read_revisions(facade: NodeFacade) -> dict[str, int | str]:
    """Read the tiny revision vector, with an old-state fallback."""

    reader = getattr(facade, "revisions", None)
    if callable(reader):
        try:
            vector = _revision_vector(reader())
        except (AttributeError, KeyError, TypeError, ValueError):
            vector = None
        if vector is not None:
            return vector
    try:
        vector = _revision_vector(facade.state())
    except (AttributeError, KeyError, TypeError, ValueError):
        vector = None
    return vector or dict(_DEFAULT_REVISION_VECTOR)


class _EventSubscriber:
    """One bounded latest-value slot owned by a connected SSE client."""

    def __init__(self) -> None:
        self.pending: tuple[int, dict[str, int | str]] | None = None
        self.closed = False


class ReviewEventHub:
    """Publish compact revision hints to a bounded set of SSE clients.

    There is no replay buffer.  Each client has one pending slot, so a burst of
    solver mutations replaces an older hint instead of queuing unbounded work.
    A reconnect receives the current vector immediately and can fetch the
    corresponding split representation with its own ETag.
    """

    def __init__(
        self,
        revision_source: object | None = None,
        *,
        revision_reader: Callable[[], object] | None = None,
        keepalive_seconds: float = 15.0,
    ) -> None:
        self._condition = threading.Condition()
        self._latest: dict[str, int | str] | None = None
        self._sequence = 0
        self._subscribers: set[_EventSubscriber] = set()
        self._closed = False
        self._keepalive_seconds = max(0.1, float(keepalive_seconds))
        self._revision_reader = revision_reader
        self._remove_source_listener: Callable[[], None] | None = None

        if self._revision_reader is None and revision_source is not None:
            reader = getattr(revision_source, "revisions", None)
            if callable(reader):
                self._revision_reader = reader
        if revision_source is not None:
            add_listener = getattr(revision_source, "add_listener", None)
            if not callable(add_listener):
                add_listener = getattr(revision_source, "subscribe", None)
            if callable(add_listener):
                try:
                    remove = add_listener(self.publish)
                except (AttributeError, TypeError, ValueError):
                    remove = None
                if callable(remove):
                    self._remove_source_listener = remove

    @property
    def subscriber_count(self) -> int:
        """Return the number of currently connected clients (for diagnostics)."""

        with self._condition:
            return len(self._subscribers)

    @property
    def latest(self) -> dict[str, int | str] | None:
        """Return a copy of the last published vector, if any."""

        with self._condition:
            return None if self._latest is None else dict(self._latest)

    @staticmethod
    def _same_epoch_at_least(
        candidate: Mapping[str, int | str],
        current: Mapping[str, int | str],
    ) -> bool:
        """Whether candidate cannot be older than current."""

        if candidate.get("session_epoch") != current.get("session_epoch"):
            return False
        return all(
            int(candidate[key]) >= int(current[key]) for key in _REVISION_KEYS[1:]
        )

    def _publish_locked(self, vector: dict[str, int | str]) -> bool:
        if self._latest == vector:
            return False
        if (
            self._latest is not None
            and vector.get("session_epoch") == self._latest.get("session_epoch")
            and not self._same_epoch_at_least(vector, self._latest)
        ):
            # Source callbacks are expected to be monotonic, but a delayed
            # transport callback must not make a connected browser go
            # backwards within one session.
            return False
        self._sequence += 1
        self._latest = dict(vector)
        for subscriber in self._subscribers:
            if not subscriber.closed:
                # Deliberately overwrite rather than append: this is the
                # bounded latest-value policy.
                subscriber.pending = (self._sequence, dict(vector))
        self._condition.notify_all()
        return True

    def publish(self, value: object) -> bool:
        """Publish a revision snapshot; duplicate vectors are ignored."""

        vector = _revision_vector(value)
        if vector is None:
            return False
        with self._condition:
            if self._closed:
                return False
            return self._publish_locked(vector)

    def _current_vector(self) -> dict[str, int | str] | None:
        """Read the source without holding the hub condition lock."""

        if self._revision_reader is not None:
            try:
                vector = _revision_vector(self._revision_reader())
            except (AttributeError, KeyError, TypeError, ValueError):
                vector = None
            return vector
        return None

    def _subscribe(self) -> _EventSubscriber:
        subscriber = _EventSubscriber()
        # A read-model revision call may take its own lock.  Never call it
        # while holding ``_condition``: a publisher can be invoked while the
        # read model lock is held, and reversing those lock orders deadlocks
        # an SSE reconnect against a concurrent solver update.
        current = self._current_vector()
        with self._condition:
            if self._closed:
                subscriber.closed = True
                return subscriber
            if current is not None and (
                self._latest is None
                or current.get("session_epoch") != self._latest.get("session_epoch")
                or self._same_epoch_at_least(current, self._latest)
            ):
                self._publish_locked(current)
            if self._latest is None:
                self._publish_locked(dict(_DEFAULT_REVISION_VECTOR))
            subscriber.pending = (
                self._sequence,
                dict(self._latest or _DEFAULT_REVISION_VECTOR),
            )
            self._subscribers.add(subscriber)
        return subscriber

    def _unsubscribe(self, subscriber: _EventSubscriber) -> None:
        with self._condition:
            subscriber.closed = True
            self._subscribers.discard(subscriber)
            self._condition.notify_all()

    def _next(self, subscriber: _EventSubscriber):
        """Return an event tuple, ``()`` for keepalive, or ``None`` on close."""

        with self._condition:
            while (
                subscriber.pending is None
                and not subscriber.closed
                and not self._closed
            ):
                self._condition.wait(self._keepalive_seconds)
                if (
                    subscriber.pending is None
                    and not subscriber.closed
                    and not self._closed
                ):
                    return ()
            if subscriber.closed or self._closed:
                return None
            item = subscriber.pending
            subscriber.pending = None
            return item

    def events(self):
        """Yield encoded SSE records until the client disconnects or hub closes."""

        subscriber = self._subscribe()
        try:
            while True:
                item = self._next(subscriber)
                if item is None:
                    return
                if item == ():
                    # The comment keeps intermediaries from timing out while
                    # the named heartbeat lets the browser watchdog distinguish
                    # a quiet stream from a stalled one.
                    yield ": keepalive\n\nevent: heartbeat\ndata: {}\n\n"
                    continue
                sequence, vector = item
                data = json.dumps(
                    vector, sort_keys=True, separators=(",", ":"), ensure_ascii=True
                )
                yield f"event: revisions\nid: {sequence}\ndata: {data}\n\n"
        finally:
            self._unsubscribe(subscriber)

    def close(self) -> None:
        """Stop new streams and wake existing streams for server shutdown."""

        with self._condition:
            if self._closed:
                return
            self._closed = True
            subscribers = tuple(self._subscribers)
            for subscriber in subscribers:
                subscriber.closed = True
            self._subscribers.clear()
            self._condition.notify_all()
        if self._remove_source_listener is not None:
            try:
                self._remove_source_listener()
            except (AttributeError, TypeError, ValueError):
                pass


def _etag_value(payload: object, kind: str) -> str:
    """Build a stable validator without changing an endpoint's JSON shape."""

    if isinstance(payload, dict):
        epoch = payload.get("session_epoch")
        if kind == "revisions":
            vector = _revision_vector(payload)
            if vector is not None:
                return "lctk-revisions-{}-{}-{}-{}".format(
                    vector["session_epoch"],
                    vector["live_revision"],
                    vector["captures_revision"],
                    vector["scene_revision"],
                )
        else:
            revision_key = {
                "state": "state_revision",
                "live": "live_revision",
                "captures": "captures_revision",
                "scene": "scene_revision",
            }.get(kind)
            revision = payload.get(revision_key) if revision_key else None
            if epoch is not None and revision is not None:
                return f"lctk-{kind}-{epoch}-{revision}"
    try:
        encoded = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            default=str,
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError):
        encoded = repr(payload).encode("utf-8")
    digest = hashlib.sha256(encoded).hexdigest()
    return f"lctk-{kind}-{digest}"


def _conditional_json(payload: dict[str, Any], kind: str) -> Response:
    """Return JSON or a bodyless 304 when the browser's validator matches."""

    etag = _etag_value(payload, kind)
    if request.if_none_match.contains(etag):
        return Response(
            status=304,
            headers={
                "ETag": f'"{etag}"',
                "Cache-Control": "private, max-age=0, must-revalidate",
            },
        )
    response = jsonify(payload)
    response.set_etag(etag)
    response.headers["Cache-Control"] = "private, max-age=0, must-revalidate"
    return response


def create_app(
    facade: NodeFacade,
    *,
    params_writable: bool = True,
    event_hub: ReviewEventHub | None = None,
    revision_source: object | None = None,
) -> Flask:
    if event_hub is None:
        if revision_source is None:
            # A production facade normally gets its read model explicitly from
            # ReviewServer.  This fallback keeps direct Flask tests and small
            # integrations useful without requiring a second object.
            revision_source = facade
        source_reader = getattr(revision_source, "revisions", None)
        event_hub = ReviewEventHub(
            revision_source,
            revision_reader=(
                source_reader
                if callable(source_reader)
                else lambda: _read_revisions(facade)
            ),
        )
    static_folder = Path(__file__).resolve().parent / "web"
    app = Flask(
        __name__,
        static_folder=str(static_folder),
        static_url_path="/static",
    )
    app.extensions["review_event_hub"] = event_hub
    # One confirmation at a time per server. The node owns the immutable export
    # snapshot; this token binds the browser's second request to its own diff.
    pending: dict[str, Any] = {"token": None, "scene_revision": None}
    pending_lock = threading.Lock()

    def scene_revision() -> int | None:
        """Read the integer scene token without letting facade failures escape."""

        try:
            value = facade.state().get("scene_revision")
        except (AttributeError, TypeError, ValueError, KeyError):
            return None
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return value

    @app.get("/")
    def index() -> Response:
        return app.send_static_file("index.html")

    @app.get("/api/state")
    def state() -> Response:
        return _conditional_json(facade.state(), "state")

    @app.get("/api/live")
    def live() -> Response:
        payload = _read_representation(
            facade,
            "live",
            (
                "mode",
                "session_epoch",
                "live_revision",
                "stillness",
                "sync",
                "identity_error",
                "stability_params",
                "params_writable",
                "params_detail",
                "export",
                "export_availability",
            ),
        )
        return _conditional_json(payload, "live")

    @app.get("/api/captures")
    def captures() -> Response:
        payload = _read_representation(
            facade,
            "captures",
            (
                "mode",
                "session_epoch",
                "captures_revision",
                "capture_revision",
                "evidence_revisions",
                "diversity",
                "solve",
                "pairs",
            ),
        )
        return _conditional_json(payload, "captures")

    @app.get("/api/revisions")
    def revisions() -> Response:
        return _conditional_json(_read_revisions(facade), "revisions")

    @app.get("/api/events")
    def events() -> Response:
        response = Response(event_hub.events(), mimetype="text/event-stream")
        response.headers["Cache-Control"] = "no-cache"
        response.headers["Connection"] = "keep-alive"
        response.headers["X-Accel-Buffering"] = "no"
        return response

    @app.get("/api/scene")
    def scene() -> Response:
        return _conditional_json(facade.scene(), "scene")

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
        with pending_lock:
            ok, detail = facade.drop(pair_id)
            if ok:
                # The diff the operator was shown described a different buffer.
                pending["token"] = None
                pending["scene_revision"] = None
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
        with pending_lock:
            pending["token"] = None
            pending["scene_revision"] = None
            before_revision = scene_revision()
            ok, detail, entry = facade.export_autoware(dry_run=True)
            token = secrets.token_urlsafe(24) if ok else None
            revision = scene_revision() if ok else None
            if ok and (before_revision is None or revision is None):
                ok = False
                detail = "cannot confirm Autoware diff: scene revision unavailable"
                entry = None
            elif ok and revision != before_revision:
                ok = False
                detail = "Autoware preview became stale; preview the diff again"
                entry = None
            if ok:
                pending["token"] = token
                pending["scene_revision"] = revision
            response = jsonify(
                {
                    "ok": ok,
                    "detail": detail,
                    "entry": entry,
                    "confirmation_token": token if ok else None,
                }
            )
            return response

    @app.post("/api/export/autoware/write")
    def autoware_write() -> Response:
        payload = request.get_json(silent=True)
        if not isinstance(payload, dict):
            payload = {}
        request_token = payload.get("confirmation_token")
        with pending_lock:
            expected_token = pending.get("token")
            expected_revision = pending.get("scene_revision")
            if expected_token is None or request_token != expected_token:
                response = jsonify(
                    {
                        "ok": False,
                        "detail": "preview the Autoware diff first; nothing is written unseen",
                        "entry": None,
                    }
                )
                return response
            # Consume valid confirmations before checking the revision or doing
            # file I/O. A failed/stale write cannot be retried unseen.
            pending["token"] = None
            pending["scene_revision"] = None
            current_revision = scene_revision()
            if current_revision != expected_revision:
                response = jsonify(
                    {
                        "ok": False,
                        "detail": "Autoware preview is stale; preview the diff again",
                        "entry": None,
                    }
                )
                return response
            ok, detail, entry = facade.export_autoware(dry_run=False)
            return jsonify({"ok": ok, "detail": detail, "entry": entry})

    return app


class ReviewServer:
    """The Flask app on a daemon thread, startable and stoppable by the node."""

    def __init__(
        self,
        facade: NodeFacade,
        *,
        host: str,
        port: int,
        revision_source: object | None = None,
    ):
        source = revision_source if revision_source is not None else facade
        source_reader = getattr(source, "revisions", None)
        self._event_hub = ReviewEventHub(
            source,
            revision_reader=(
                source_reader
                if callable(source_reader)
                else lambda: _read_revisions(facade)
            ),
        )
        self._server = make_server(
            host,
            port,
            create_app(
                facade,
                params_writable=is_loopback_host(host),
                event_hub=self._event_hub,
            ),
            threaded=True,
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever, name="lctk-review", daemon=True
        )

    @property
    def port(self) -> int:
        return self._server.server_port

    @property
    def event_hub(self) -> ReviewEventHub:
        """Expose the hub for diagnostics and lifecycle tests."""

        return self._event_hub

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        self._event_hub.close()
        self._server.shutdown()
        self._thread.join(timeout=2.0)


__all__ = [
    "NodeFacade",
    "ReviewEventHub",
    "ReviewServer",
    "create_app",
    "is_loopback_host",
    "validate_stability_params",
]
