"""The review server, exercised through Flask's test client.

The whole point of the NodeFacade seam is that these tests need no ROS graph, no
node, and no camera. If a test here needs rclpy, the seam has leaked.
"""

import json

import pytest
from lidar_to_camera_solver.review_server import create_app


class FakeFacade:
    def __init__(self):
        self.dropped = []
        self.exported = []
        self.autoware_calls = []
        self._state = {
            "mode": "assisted",
            "sync": "sync: groups=12",
            "stillness": {"is_still": True, "reason": "held still", "frames": 5},
            "diversity": {"n_placements": 2, "shortfalls": ["move the board"]},
            "solve": {"status": "solved", "rms_px": 0.5},
            "pairs": [{"id": 1, "rms_px": 0.5, "has_preview": True}],
            "export": {"archive_path": "/tmp/detections.json", "autoware_ready": True},
        }
        self._previews = {1: b"\xff\xd8fakejpeg\xff\xd9"}

    def state(self):
        return self._state

    def preview(self, pair_id):
        return self._previews.get(pair_id)

    def drop(self, pair_id):
        if pair_id not in self._previews:
            return False, f"no pair {pair_id}"
        self.dropped.append(pair_id)
        return True, "dropped"

    def export_archive(self, path):
        self.exported.append(path)
        return True, f"wrote {path}"

    def export_autoware(self, dry_run):
        self.autoware_calls.append(dry_run)
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


def test_state_is_returned_verbatim(client):
    response = client.get("/api/state")
    assert response.status_code == 200
    assert json.loads(response.data) == client.facade.state()


def test_preview_returns_jpeg(client):
    response = client.get("/api/pair/1/preview.jpg")
    assert response.status_code == 200
    assert response.mimetype == "image/jpeg"
    assert response.data.startswith(b"\xff\xd8")


def test_missing_preview_is_404_not_500(client):
    assert client.get("/api/pair/99/preview.jpg").status_code == 404


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
    assert client.facade.autoware_calls == [True], "preview must be a dry run"


def test_autoware_write_is_refused_before_a_preview(client):
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False
    assert "preview" in payload["detail"].lower()
    assert client.facade.autoware_calls == [], "nothing may be written unseen"


def test_autoware_write_is_allowed_after_a_preview(client):
    client.post("/api/export/autoware/preview")
    response = client.post("/api/export/autoware/write")
    assert json.loads(response.data)["ok"] is True
    assert client.facade.autoware_calls == [True, False]


def test_a_drop_invalidates_a_pending_autoware_confirmation(client):
    client.post("/api/export/autoware/preview")
    client.post("/api/pair/1/drop")
    response = client.post("/api/export/autoware/write")
    payload = json.loads(response.data)
    assert payload["ok"] is False, (
        "the buffer changed after the diff was shown, so the confirmation is stale"
    )
    assert client.facade.autoware_calls == [True]
