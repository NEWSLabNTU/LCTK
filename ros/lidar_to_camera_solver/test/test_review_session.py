"""Pure browser-session tests; no DOM or ROS graph required."""

import subprocess
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / "lidar_to_camera_solver" / "web"


def run_module(script):
    return subprocess.run(
        ["node", "--input-type=module", "--eval", script],
        cwd=WEB,
        capture_output=True,
        text=True,
        check=False,
    )


def test_first_state_paints_before_scene_or_cloud_hydration():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        let sceneCalled = false;
        let cloudCalled = false;
        const events = [];
        const api = {
          state: async () => ({ok: true, etag: 'state-1', payload: {
            session_epoch: 'epoch-1', state_revision: 1, capture_revision: 1,
            scene_revision: 1, pairs: [{id: 3, has_preview: false, evidence_revision: 1}],
          }}),
          scene: async () => {
            sceneCalled = true;
            return new Promise(() => {});
          },
          cloud: async () => {
            cloudCalled = true;
            return new Promise(() => {});
          },
          preview: async () => null,
        };
        const session = new ReviewSession(api, {
          onChange: (dirty, app) => events.push({dirty, hasState: app.state != null}),
        });
        await session.start();
        if (!events.length || !events[0].hasState || !events[0].dirty.state) {
          throw new Error('state did not paint immediately');
        }
        if (!sceneCalled) throw new Error('scene hydration was not scheduled');
        if (session.app.state.pairs.length !== 1) throw new Error('pair missing after first paint');
        // No scene payload means cloud work waits for scene geometry.
        if (cloudCalled) throw new Error('cloud request blocked first paint');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_preview_cache_reuses_bytes_and_discards_stale_selection_results():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        const pending = new Map();
        const calls = new Map();
        const api = {
          state: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', state_revision: 1, capture_revision: 1,
            scene_revision: 1,
            pairs: [
              {id: 1, has_preview: true, evidence_revision: 2},
              {id: 2, has_preview: true, evidence_revision: 1},
            ],
          }}),
          scene: async () => ({ok: true, payload: {scene_revision: 1, captures: []}}),
          cloud: async () => null,
          preview: (id) => {
            calls.set(id, (calls.get(id) || 0) + 1);
            if (id === 1 && !pending.has(id)) {
              pending.set(id, {});
              return new Promise((resolve) => { pending.get(id).resolve = resolve; });
            }
            return Promise.resolve(new Blob([String(id)]));
          },
        };
        const urls = {next: 0, revoked: [], createObjectURL: () => `blob:${++urls.next}`, revokeObjectURL: (url) => urls.revoked.push(url)};
        const session = new ReviewSession(api, {urlApi: urls});
        await session.start();
        session.select(1);
        session.select(2);
        await new Promise((resolve) => setTimeout(resolve, 0));
        if (session.app.preview.id !== 2 || session.app.preview.status !== 'ready') {
          throw new Error('new selection was not shown');
        }
        pending.get(1).resolve(new Blob(['one']));
        await new Promise((resolve) => setTimeout(resolve, 0));
        if (session.app.preview.id !== 2) throw new Error('stale preview replaced selection');
        session.select(1);
        await new Promise((resolve) => setTimeout(resolve, 0));
        if (session.app.preview.id !== 1 || session.app.preview.status !== 'ready') {
          throw new Error('cached preview did not attach');
        }
        if (calls.get(1) !== 1) throw new Error(`preview fetched ${calls.get(1)} times`);
        if (!urls.revoked.length) throw new Error('displayed object URL was not revoked');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_matching_state_304_keeps_existing_payload_without_a_second_paint():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        let reads = 0;
        let paints = 0;
        const payload = {session_epoch: 'epoch-1', state_revision: 4,
          capture_revision: 2, scene_revision: 2, pairs: []};
        const api = {
          state: async (etag) => {
            reads += 1;
            return reads === 1
              ? {ok: true, etag: '"state-4"', payload}
              : {ok: true, notModified: true, etag: '"state-4"'};
          },
          scene: async () => new Promise(() => {}),
          cloud: async () => null,
        };
        const session = new ReviewSession(api, {onChange: () => paints += 1});
        await session.start();
        await session.poll();
        if (reads !== 2 || paints !== 1) throw new Error(`304 repainted: ${reads}/${paints}`);
        if (session.app.state !== payload) throw new Error('304 discarded state');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_cloud_hydration_limits_concurrency_across_overlapping_heartbeats():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        const waiting = [];
        let active = 0;
        let maximum = 0;
        let calls = 0;
        const api = {
          cloud: (id) => {
            calls += 1;
            active += 1;
            maximum = Math.max(maximum, active);
            return new Promise((resolve) => waiting.push(() => {
              active -= 1;
              resolve(new ArrayBuffer(12));
            }));
          },
        };
        const session = new ReviewSession(api, { cloudConcurrency: 4 });
        session.epoch = 'epoch-1';
        session.app.state = {
          pairs: Array.from({length: 6}, (_, id) => ({id, evidence_revision: 1})),
        };
        session.app.scene = {
          scene_revision: 1,
          captures: Array.from({length: 6}, (_, id) => ({id})),
        };
        const first = session.hydrateClouds();
        const second = session.hydrateClouds();
        while (calls < 6) {
          await new Promise((resolve) => setTimeout(resolve, 0));
          while (waiting.length) waiting.shift()();
        }
        await Promise.all([first, second]);
        if (maximum > 4) throw new Error(`cloud concurrency exceeded: ${maximum}`);
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout
