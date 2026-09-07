"""The review server, exercised through Flask's test client.

The whole point of the NodeFacade seam is that these tests need no ROS graph, no
node, and no camera. If a test here needs rclpy, the seam has leaked.
"""

import json
import subprocess
from pathlib import Path

import pytest
from lidar_to_camera_solver.review_server import create_app, is_loopback_host


@pytest.mark.parametrize(
    ("host", "expected"),
    [
        ("127.0.0.1", True),
        ("::1", True),
        ("0.0.0.0", False),
        ("::", False),
        ("192.168.1.10", False),
        ("localhost", False),
    ],
)
def test_parameter_write_gate_accepts_only_numeric_loopback_addresses(host, expected):
    assert is_loopback_host(host) is expected


class FakeFacade:
    def __init__(self):
        self.dropped = []
        self.exported = []
        self.autoware_calls = []
        self.mutate_during_preview = False
        self.param_calls = []
        self._state = {
            "mode": "assisted",
            "scene_revision": 4,
            "sync": "sync: groups=12",
            "stillness": {"is_still": True, "reason": "held still", "frames": 5},
            "diversity": {"n_placements": 2, "shortfalls": ["move the board"]},
            "solve": {"status": "solved", "rms_px": 0.5},
            "pairs": [{"id": 1, "rms_px": 0.5, "has_preview": True}],
            "export": {"archive_path": "/tmp/detections.json", "autoware_ready": True},
            "stability_params": {
                "stability_window_s": 1.0,
                "stability_max_translation_m": 0.005,
                "stability_max_rotation_deg": 0.5,
            },
        }
        self._previews = {1: b"\xff\xd8fakejpeg\xff\xd9"}
        self._clouds = {1: b"\x00\x00\x80?\x00\x00\x00@\x00\x00@@"}
        self._scene = {
            "scene_revision": 4,
            "world_frame_id": "velodyne",
            "captures": [],
            "camera": None,
        }

    def state(self):
        return self._state

    def preview(self, pair_id):
        return self._previews.get(pair_id)

    def cloud(self, pair_id):
        return self._clouds.get(pair_id)

    def scene(self):
        return self._scene

    def set_stability_params(self, values):
        self.param_calls.append(values)
        self._state["stability_params"].update(values)
        return True, "updated"

    def drop(self, pair_id):
        if pair_id not in self._previews:
            return False, f"no pair {pair_id}"
        self.dropped.append(pair_id)
        self._state["scene_revision"] += 1
        return True, "dropped"

    def export_archive(self, path):
        self.exported.append(path)
        return True, f"wrote {path}"

    def export_autoware(self, dry_run):
        self.autoware_calls.append(dry_run)
        if dry_run and self.mutate_during_preview:
            self._state["scene_revision"] += 1
        return True, "ok", {"x": 1.0, "y": 2.0}


@pytest.fixture
def client():
    facade = FakeFacade()
    app = create_app(facade)
    app.config["TESTING"] = True
    with app.test_client() as test_client:
        test_client.facade = facade
        yield test_client


def test_index_serves_a_self_contained_page(client):
    response = client.get("/")
    assert response.status_code == 200
    body = response.data.decode()
    assert "<html" in body.lower()
    assert "http://" not in body.replace("http://www.w3.org", ""), (
        "the page must not reference any external host; the rig has no internet"
    )
    assert "scene.js" not in body, "the throwaway canvas mock must not ship"
    assert 'type="module"' in body
    assert "/static/main.js" in body
    assert "/static/vendor/three.module.js" not in body, (
        "three.js is imported by the local scene module, not from a CDN"
    )


def test_index_uses_the_image_host_and_keeps_action_text_readable(client):
    body = client.get("/").data.decode()
    assert 'id="preview"' not in body
    assert 'id="preview-host"' not in body, "Chrome owns the dynamic preview host"
    assert "text-overflow: ellipsis" not in body
    assert "overflow-wrap: anywhere" in body
    assert 'id="paramReset"' in body
    assert "captured at (camera clock)" in body


def test_index_serves_the_local_frontend_modules(client):
    assert client.get("/static/main.js").status_code == 200
    assert client.get("/static/chrome.js").status_code == 200
    assert client.get("/static/scene_model.js").status_code == 200
    assert client.get("/static/review_api.js").status_code == 200
    assert client.get("/static/review_session.js").status_code == 200
    assert client.get("/static/quality.js").status_code == 200
    assert client.get("/static/vendor/three.module.js").status_code == 200


def test_scene_model_has_world_reference_and_observed_sizing(client):
    source = client.get("/static/scene_model.js").data.decode()
    assert "LiDAR XY reference" in source
    assert "ResizeObserver" in source
    assert "camera optical" in source


def test_scene_model_uses_autoware_z_up_orbit_without_a_pole_wall(client):
    source = client.get("/static/scene_model.js").data.decode()
    assert "this.camera.up.set(0, 0, 1)" in source
    assert "this._orbitPitch += dy * 0.006" in source
    assert "Math.cos(this._orbitPitch)" in source
    assert "Math.max(0.08" not in source


def test_frontend_orbit_crosses_the_sky_without_a_half_turn_flip():
    result = _run_frontend_module(
        """
        import * as THREE from './vendor/three.module.js';
        import { SceneModel } from './scene_model.js';

        const model = Object.create(SceneModel.prototype);
        model.target = new THREE.Vector3();
        model.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
        model.camera.up.set(0, 0, 1);
        model._orbitYaw = 0.4;
        model._orbitPitch = Math.PI / 2 - 0.03;
        model._orbitRadius = 3;
        model._setCameraFromOrbit();
        const before = model.camera.quaternion.clone();

        // A small downward drag crosses the sky-facing pole.
        model._orbit(0, 10);
        const after = model.camera.quaternion.clone();
        if (before.angleTo(after) > 0.2) {
          throw new Error(`orbit flipped ${before.angleTo(after)} radians`);
        }
        if (model.camera.position.distanceTo(model.target) < 2.99) {
          throw new Error('orbit radius changed while crossing the pole');
        }
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_frontend_orbit_downward_drag_raises_camera_elevation():
    result = _run_frontend_module(
        """
        import * as THREE from './vendor/three.module.js';
        import { SceneModel } from './scene_model.js';

        const model = Object.create(SceneModel.prototype);
        model.target = new THREE.Vector3();
        model.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
        model.camera.up.set(0, 0, 1);
        model._orbitYaw = 0.4;
        model._orbitPitch = 0.35;
        model._orbitRadius = 3;
        model._setCameraFromOrbit();
        const beforeZ = model.camera.position.z;

        // Downward pointer motion must raise the camera in elevation.
        model._orbit(0, 10);
        if (!(model.camera.position.z > beforeZ)) {
          throw new Error('downward drag lowered camera elevation');
        }
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_frontend_orbit_zoom_keeps_an_over_pole_orientation():
    result = _run_frontend_module(
        """
        import * as THREE from './vendor/three.module.js';
        import { SceneModel } from './scene_model.js';

        const model = Object.create(SceneModel.prototype);
        model.target = new THREE.Vector3();
        model.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
        model.camera.up.set(0, 0, 1);
        model._orbitYaw = -0.7;
        model._orbitPitch = Math.PI / 2 + 0.4;
        model._orbitRadius = 3;
        model._setCameraFromOrbit();
        const before = model.camera.quaternion.clone();
        model._zoom(1.25);
        if (Math.abs(model._orbitPitch - (Math.PI / 2 + 0.4)) > 1e-12) {
          throw new Error('zoom canonicalized the over-pole pitch');
        }
        if (Math.abs(model.camera.position.distanceTo(model.target) - 3.75) > 1e-9) {
          throw new Error('zoom changed the wrong orbit radius');
        }
        if (before.angleTo(model.camera.quaternion) > 1e-9) {
          throw new Error('zoom changed the over-pole orientation');
        }
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_advanced_parameter_drafts_have_a_distinct_dirty_style(client):
    index = client.get("/").data.decode()
    chrome = client.get("/static/chrome.js").data.decode()
    assert ".param input.dirty" in index
    assert ".reset.dirty" in index
    assert 'classList.toggle("dirty"' in chrome


def _run_frontend_module(script):
    web = Path(__file__).resolve().parents[1] / "lidar_to_camera_solver" / "web"
    return subprocess.run(
        ["node", "--input-type=module", "--eval", script],
        cwd=web,
        capture_output=True,
        text=True,
        check=False,
    )


def test_frontend_selection_keeps_null_distinct_from_capture_zero():
    result = _run_frontend_module(
        """
        import { pairId } from './chrome.js';
        for (const value of [null, undefined, '', '  ', true, false, -1, 'bad']) {
          if (pairId(value) !== null) throw new Error(`accepted invalid id: ${value}`);
        }
        if (pairId(0) !== 0 || pairId('0') !== 0) throw new Error('rejected capture zero');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_frontend_rms_bands_match_the_sidebar_palette():
    result = _run_frontend_module(
        """
        import { rmsBand, rmsColorHex } from './quality.js';
        const cases = [
          [null, null, 0x8c98a8], [4.999, 'low', 0x4bcf7d],
          [5, 'medium', 0xf0a53a], [9.999, 'medium', 0xf0a53a],
          [10, 'high', 0xef5f80],
        ];
        for (const [value, band, color] of cases) {
          if (rmsBand(value) !== band) throw new Error(`bad band for ${value}`);
          if (rmsColorHex(value) !== color) throw new Error(`bad color for ${value}`);
        }
        if (rmsColorHex(5, false) !== 0x8c98a8) throw new Error('toggle ignored');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_index_is_the_packaged_static_asset(client):
    source = (
        Path(__file__).resolve().parents[1]
        / "lidar_to_camera_solver"
        / "web"
        / "index.html"
    )
    index = client.get("/")
    static = client.get("/static/index.html")

    assert index.data == source.read_bytes()
    assert static.data == index.data
    assert static.mimetype == "text/html"


def test_state_is_returned_verbatim(client):
    response = client.get("/api/state")
    assert response.status_code == 200
    assert json.loads(response.data) == client.facade.state()


def test_state_supports_conditional_requests_without_changing_the_json_shape(client):
    first = client.get("/api/state")
    assert first.status_code == 200
    etag = first.headers.get("ETag")
    assert etag

    second = client.get("/api/state", headers={"If-None-Match": etag})

    assert second.status_code == 304
    assert second.data == b""
    assert second.headers.get("ETag") == etag


def test_scene_supports_conditional_requests(client):
    first = client.get("/api/scene")
    etag = first.headers.get("ETag")
    assert first.status_code == 200 and etag

    second = client.get("/api/scene", headers={"If-None-Match": etag})

    assert second.status_code == 304
    assert second.data == b""
    assert second.headers.get("ETag") == etag


def test_changed_state_revision_returns_a_new_representation(client):
    first = client.get("/api/state")
    etag = first.headers["ETag"]
    client.facade._state["stillness"] = {
        "is_still": False,
        "reason": "moving",
        "frames": 6,
    }

    second = client.get("/api/state", headers={"If-None-Match": etag})

    assert second.status_code == 200
    assert second.headers["ETag"] != etag


def test_scene_is_returned_verbatim(client):
    response = client.get("/api/scene")
    assert response.status_code == 200
    assert json.loads(response.data) == client.facade.scene()


def test_params_updates_whitelisted_values_and_returns_effective_set(client):
    response = client.post(
        "/api/params",
        data=json.dumps({"stability_window_s": 2.5}),
        content_type="application/json",
    )
    payload = json.loads(response.data)
    assert response.status_code == 200
    assert payload["ok"] is True
    assert payload["params"] == client.facade.state()["stability_params"]
    assert client.facade.param_calls == [{"stability_window_s": 2.5}]


def test_params_reject_unknown_key_atomically(client):
    response = client.post(
        "/api/params",
        data=json.dumps({"stability_window_s": 2.5, "publishing_rate": 1.0}),
        content_type="application/json",
    )
    payload = json.loads(response.data)
    assert response.status_code == 400
    assert payload["ok"] is False
    assert "publishing_rate" in payload["detail"]
    assert client.facade.param_calls == []


@pytest.mark.parametrize(
    "value", [0, -1, True, "1.0", None, float("nan"), float("inf"), 10**400]
)
def test_params_reject_non_positive_or_non_numeric_values(client, value):
    response = client.post(
        "/api/params",
        data=json.dumps({"stability_window_s": value}),
        content_type="application/json",
    )
    assert response.status_code == 400
    assert json.loads(response.data)["ok"] is False
    assert client.facade.param_calls == []


def test_params_are_refused_when_server_is_not_loopback():
    facade = FakeFacade()
    app = create_app(facade, params_writable=False)
    app.config["TESTING"] = True
    with app.test_client() as test_client:
        response = test_client.post(
            "/api/params",
            data=json.dumps({"stability_window_s": 2.5}),
            content_type="application/json",
        )
    assert response.status_code == 403
    assert "loopback" in json.loads(response.data)["detail"]
    assert facade.param_calls == []


def test_preview_returns_jpeg(client):
    response = client.get("/api/pair/1/preview.jpg")
    assert response.status_code == 200
    assert response.mimetype == "image/jpeg"
    assert response.data.startswith(b"\xff\xd8")


def test_missing_preview_is_404_not_500(client):
    assert client.get("/api/pair/99/preview.jpg").status_code == 404


def test_cloud_returns_packed_binary(client):
    response = client.get("/api/pair/1/cloud.bin")
    assert response.status_code == 200
    assert response.mimetype == "application/octet-stream"
    assert response.data == client.facade._clouds[1]


def test_missing_cloud_is_404_not_a_neighbouring_sweep(client):
    assert client.get("/api/pair/99/cloud.bin").status_code == 404


def test_drop_calls_the_facade(client):
    response = client.post("/api/pair/1/drop")
    assert response.status_code == 200
    assert json.loads(response.data)["ok"] is True
    assert client.facade.dropped == [1]


def test_drop_of_an_unknown_pair_reports_failure_without_raising(client):
    response = client.post("/api/pair/99/drop")
    assert response.status_code == 200
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "99" in payload["detail"]


def test_export_archive_passes_the_path(client):
    response = client.post(
        "/api/export/archive",
        data=json.dumps({"path": "/tmp/out.json"}),
        content_type="application/json",
    )
    assert json.loads(response.data)["ok"] is True
    assert client.facade.exported == ["/tmp/out.json"]


def test_export_archive_requires_a_path(client):
    response = client.post(
        "/api/export/archive", data=json.dumps({}), content_type="application/json"
    )
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "path" in payload["detail"]


def test_autoware_preview_does_not_write(client):
    response = client.post("/api/export/autoware/preview")
    payload = json.loads(response.data)
    assert payload["ok"] is True
    assert payload["entry"] == {"x": 1.0, "y": 2.0}
    assert payload["confirmation_token"]
    assert client.facade.autoware_calls == [True], "preview must be a dry run"


def test_autoware_write_is_refused_before_a_preview(client):
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "preview" in payload["detail"].lower()
    assert client.facade.autoware_calls == [], "nothing may be written unseen"


def test_autoware_write_is_allowed_after_a_preview(client):
    preview = client.post("/api/export/autoware/preview")
    token = json.loads(preview.data)["confirmation_token"]
    response = client.post(
        "/api/export/autoware/write",
        data=json.dumps({"confirmation_token": token}),
        content_type="application/json",
    )
    assert json.loads(response.data)["ok"] is True
    assert client.facade.autoware_calls == [True, False]


def test_autoware_write_requires_the_explicit_preview_token(client):
    client.post("/api/export/autoware/preview")
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert client.facade.autoware_calls == [True]


def test_autoware_write_rejects_a_confirmation_from_another_preview(client):
    preview = client.post("/api/export/autoware/preview")
    token = json.loads(preview.data)["confirmation_token"]
    response = client.post(
        "/api/export/autoware/write",
        data=json.dumps({"confirmation_token": "not-the-preview-token"}),
        content_type="application/json",
    )
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "preview" in payload["detail"].lower()
    assert client.facade.autoware_calls == [True]
    response = client.post(
        "/api/export/autoware/write",
        data=json.dumps({"confirmation_token": token}),
        content_type="application/json",
    )
    assert json.loads(response.data)["ok"] is True
    assert client.facade.autoware_calls == [True, False]


def test_autoware_write_rejects_a_scene_mutation_between_requests(client):
    preview = client.post("/api/export/autoware/preview")
    token = json.loads(preview.data)["confirmation_token"]
    client.facade._state["scene_revision"] += 1
    response = client.post(
        "/api/export/autoware/write",
        data=json.dumps({"confirmation_token": token}),
        content_type="application/json",
    )
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "stale" in payload["detail"].lower()
    assert client.facade.autoware_calls == [True]


def test_autoware_preview_refuses_when_scene_changes_during_dry_run(client):
    client.facade.mutate_during_preview = True
    response = client.post("/api/export/autoware/preview")
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "stale" in payload["detail"].lower()
    assert payload["confirmation_token"] is None


def test_a_drop_invalidates_a_pending_autoware_confirmation(client):
    client.post("/api/export/autoware/preview")
    client.post("/api/pair/1/drop")
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False, (
        "the buffer changed after the diff was shown, so the confirmation is stale"
    )
    assert client.facade.autoware_calls == [True]
