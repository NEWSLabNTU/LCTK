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


def test_revisions_fallback_refreshes_only_the_changed_projection():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        let revisionCalls = 0;
        let liveCalls = 0;
        let capturesCalls = 0;
        const api = {
          state: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', state_revision: 1, live_revision: 1,
            captures_revision: 1, capture_revision: 1, scene_revision: 1,
            stillness: {is_still: false}, pairs: [{id: 4, evidence_revision: 1}],
          }}),
          openEvents: () => null,
          revisions: async () => {
            revisionCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', live_revision: revisionCalls > 1 ? 2 : 1,
              captures_revision: 1, scene_revision: 1,
            }};
          },
          live: async () => {
            liveCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', live_revision: 2,
              stillness: {is_still: true},
            }};
          },
          captures: async () => {
            capturesCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', captures_revision: 1,
              pairs: [{id: 4, evidence_revision: 1}],
            }};
          },
          scene: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', scene_revision: 1, captures: [],
          }}),
          cloud: async () => null,
        };
        const session = new ReviewSession(api, {revisionsRetryMs: 100});
        await session.start();
        await new Promise((resolve) => setTimeout(resolve, 250));
        session.stopEvents();
        if (revisionCalls < 2) throw new Error(`fallback polled ${revisionCalls} times`);
        if (liveCalls !== 1) throw new Error(`live fetched ${liveCalls} times`);
        if (capturesCalls !== 0) throw new Error(`captures fetched ${capturesCalls} times`);
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


def test_live_only_sse_hint_does_not_fetch_captures_projection():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        class Source {
          constructor() { this.listeners = new Map(); }
          addEventListener(name, callback) {
            const list = this.listeners.get(name) || [];
            list.push(callback);
            this.listeners.set(name, list);
          }
          emit(name, data) {
            for (const callback of this.listeners.get(name) || []) callback({data: JSON.stringify(data)});
          }
          close() {}
        }
        const source = new Source();
        let liveCalls = 0;
        let capturesCalls = 0;
        const api = {
          state: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', state_revision: 1, live_revision: 1,
            captures_revision: 1, capture_revision: 1, scene_revision: 1,
            stillness: {is_still: false}, sync: 'sync: groups=1',
            pairs: [{id: 4, evidence_revision: 1}],
          }}),
          live: async () => {
            liveCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', live_revision: 2,
              stillness: {is_still: true}, sync: 'sync: groups=2',
            }};
          },
          captures: async () => {
            capturesCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', captures_revision: 1,
              capture_revision: 1, pairs: [{id: 4, evidence_revision: 1}],
            }};
          },
          revisions: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', live_revision: 1,
            captures_revision: 1, scene_revision: 1,
          }}),
          scene: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', scene_revision: 1, captures: [],
          }}),
          openEvents: () => source,
          cloud: async () => null,
        };
        const session = new ReviewSession(api);
        await session.start();
        source.emit('revisions', {
          session_epoch: 'epoch-1', live_revision: 2,
          captures_revision: 1, scene_revision: 1,
        });
        await new Promise((resolve) => setTimeout(resolve, 0));
        session.stopEvents();
        if (liveCalls !== 1) throw new Error(`live fetched ${liveCalls} times`);
        if (capturesCalls !== 0) throw new Error(`captures fetched ${capturesCalls} times`);
        if (session.app.captures.pairs.length !== 1) throw new Error('capture list changed');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_captures_sse_hint_fetches_only_capture_projection():
    result = run_module(
        """
        import { ReviewSession } from './review_session.js';
        class Source {
          constructor() { this.listeners = new Map(); }
          addEventListener(name, callback) {
            const list = this.listeners.get(name) || [];
            list.push(callback);
            this.listeners.set(name, list);
          }
          emit(name, data) {
            for (const callback of this.listeners.get(name) || []) callback({data: JSON.stringify(data)});
          }
          close() {}
        }
        const source = new Source();
        let capturesCalls = 0;
        let liveCalls = 0;
        const api = {
          state: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', state_revision: 1, live_revision: 1,
            captures_revision: 1, capture_revision: 1, scene_revision: 1,
            pairs: [{id: 4, evidence_revision: 1}],
          }}),
          live: async () => { liveCalls += 1; return {ok: true, payload: {session_epoch: 'epoch-1', live_revision: 1}}; },
          captures: async () => {
            capturesCalls += 1;
            return {ok: true, payload: {
              session_epoch: 'epoch-1', captures_revision: 2,
              capture_revision: 2, pairs: [{id: 4, evidence_revision: 2}, {id: 5, evidence_revision: 1}],
            }};
          },
          revisions: async () => ({ok: true, payload: {
            session_epoch: 'epoch-1', live_revision: 1,
            captures_revision: 1, scene_revision: 1,
          }}),
          scene: async () => ({ok: true, payload: {session_epoch: 'epoch-1', scene_revision: 1, captures: []}}),
          openEvents: () => source,
          cloud: async () => null,
        };
        const session = new ReviewSession(api);
        await session.start();
        source.emit('revisions', {
          session_epoch: 'epoch-1', live_revision: 1,
          captures_revision: 2, scene_revision: 1,
        });
        await new Promise((resolve) => setTimeout(resolve, 0));
        session.stopEvents();
        if (capturesCalls !== 1) throw new Error(`captures fetched ${capturesCalls} times`);
        if (liveCalls !== 0) throw new Error(`live fetched ${liveCalls} times`);
        if (session.app.captures.pairs.length !== 2) throw new Error('capture projection not applied');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_load_archive_replaces_review_and_invalidates_old_evidence_requests():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewSession } from './review_session.js';
        let loaded = false;
        let resolvePreview;
        let closed = 0;
        let opened = 0;
        const stateEtags = [];
        const api = {
          autowareToken: 'old-confirmation',
          state: async (etag) => {
            stateEtags.push(etag);
            return {ok: true, etag: loaded ? 'new' : 'old', payload: {
              session_epoch: 'same-epoch', scene_revision: loaded ? 2 : 1,
              captures_revision: loaded ? 2 : 1,
              pairs: [{id: loaded ? 9 : 3, evidence_revision: 1, has_preview: !loaded}],
            }};
          },
          scene: async () => ({ok: true, payload: {scene_revision: loaded ? 2 : 1, captures: []}}),
          preview: () => new Promise((resolve) => { resolvePreview = resolve; }),
          loadArchive: async (path) => {
            assert.equal(path, '/srv/archive.json');
            loaded = true;
            return {ok: true, detail: 'Loaded captures'};
          },
          openEvents: () => { opened += 1; return {close: () => {closed += 1;}}; },
        };
        const session = new ReviewSession(api);
        await session.start();
        session.select(3);
        session.app.autowarePreview = {old: true};
        const result = await session.loadArchive('/srv/archive.json');
        resolvePreview(new Blob(['old image']));
        await new Promise((resolve) => setTimeout(resolve, 0));
        assert.equal(result.ok, true);
        assert.deepEqual(session.app.state.pairs.map((pair) => pair.id), [9]);
        assert.equal(session.app.selectedId, null);
        assert.equal(session.app.preview.status, 'idle');
        assert.equal(session.app.clouds.size, 0);
        assert.equal(session.app.autowarePreview, null);
        assert.equal(api.autowareToken, null);
        assert.deepEqual(stateEtags, [null, null]);
        assert.equal(closed, 1);
        assert.equal(opened, 2);
        session.stopEvents();
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_failed_archive_load_preserves_review_and_confirmation():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewSession } from './review_session.js';
        let stateCalls = 0;
        const api = {
          autowareToken: 'keep-confirmation',
          state: async () => { stateCalls += 1; return {ok: true, payload: {pairs: [{id: 3}]}}; },
          scene: async () => ({ok: true, payload: {captures: []}}),
          loadArchive: async () => ({ok: false, detail: 'Wrong Target Identity'}),
        };
        const session = new ReviewSession(api);
        await session.refreshState();
        session.select(3);
        session.app.autowarePreview = {keep: true};
        const state = session.app.state;
        const result = await session.loadArchive('/srv/wrong.json');
        assert.equal(result.ok, false);
        assert.equal(session.app.state, state);
        assert.equal(session.app.selectedId, 3);
        assert.deepEqual(session.app.autowarePreview, {keep: true});
        assert.equal(api.autowareToken, 'keep-confirmation');
        assert.equal(stateCalls, 1);
        session.stopEvents();
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_committed_archive_with_failed_refresh_reports_and_retries_bootstrap():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewSession } from './review_session.js';
        let loaded = false;
        let refreshAttempts = 0;
        let opened = 0;
        const api = {
          autowareToken: 'old-confirmation',
          state: async () => {
            if (loaded && ++refreshAttempts === 1) return {ok: false, detail: 'Connection lost'};
            return {ok: true, payload: {pairs: [{id: loaded ? 9 : 3}]}};
          },
          scene: async () => ({ok: true, payload: {captures: []}}),
          loadArchive: async () => { loaded = true; return {ok: true, detail: 'Loaded captures'}; },
          openEvents: () => { opened += 1; return {close() {}}; },
        };
        const session = new ReviewSession(api, {revisionsRetryMs: 100});
        await session.start();
        session.app.autowarePreview = {old: true};
        const result = await session.loadArchive('/srv/archive.json');
        assert.equal(result.ok, true, 'the archive replacement already committed');
        assert.match(result.detail, /archive loaded.*review refresh failed/i);
        assert.equal(session.app.state, null);
        assert.equal(session.app.autowarePreview, null);
        assert.equal(api.autowareToken, null);
        assert.equal(opened, 1, 'events wait for the new authoritative state');
        await new Promise((resolve) => setTimeout(resolve, 160));
        assert.equal(session.app.state.pairs[0].id, 9);
        assert.equal(opened, 2);
        session.stopEvents();
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_archive_cloud_hydration_does_not_wait_for_previous_buffer_clouds():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewSession } from './review_session.js';
        let loaded = false;
        let resolveOldCloud;
        const cloudCalls = [];
        const api = {
          state: async () => ({ok: true, payload: {session_epoch: 'same-epoch',
            scene_revision: loaded ? 2 : 1, pairs: [{id: loaded ? 9 : 3, evidence_revision: 1}]}}),
          scene: async () => ({ok: true, payload: {session_epoch: 'same-epoch',
            scene_revision: loaded ? 2 : 1, captures: [{id: loaded ? 9 : 3}]}}),
          cloud: (id) => {
            cloudCalls.push(id);
            return id === 3 ? new Promise((resolve) => {resolveOldCloud = resolve;})
              : Promise.resolve(new ArrayBuffer(12));
          },
          loadArchive: async () => {loaded = true; return {ok: true, detail: 'Loaded'};},
        };
        const session = new ReviewSession(api);
        await session.refreshState();
        await new Promise((resolve) => setTimeout(resolve, 0));
        void session.hydrateClouds();
        await session.loadArchive('/srv/archive.json');
        await new Promise((resolve) => setTimeout(resolve, 0));
        void session.hydrateClouds();
        await new Promise((resolve) => setTimeout(resolve, 0));
        assert.deepEqual(cloudCalls, [3, 9], 'new evidence must not wait for old requests');
        resolveOldCloud(new ArrayBuffer(12));
        await new Promise((resolve) => setTimeout(resolve, 0));
        assert.equal(session.app.clouds.has(3), false);
        assert.equal(session.app.clouds.has(9), true);
        session.stopEvents();
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_rejected_preload_projection_requests_cannot_change_restored_review():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewSession } from './review_session.js';
        for (const kind of ['live', 'captures', 'scene']) {
          let loaded = false;
          let rejectOld;
          let sceneCalls = 0;
          const scheduledRetries = [];
          globalThis.setTimeout = (callback, delay) => {
            scheduledRetries.push({callback, delay});
            return scheduledRetries.length;
          };
          globalThis.clearTimeout = () => {};
          const api = {
            state: async () => ({ok: true, payload: {session_epoch: 'same-epoch',
              live_revision: loaded ? 2 : 1, captures_revision: loaded ? 2 : 1,
              scene_revision: loaded ? 2 : 1, pairs: [{id: loaded ? 9 : 3}]}}),
            scene: () => {
              sceneCalls += 1;
              if (kind === 'scene' && sceneCalls === 1) {
                return new Promise((resolve, reject) => {rejectOld = reject;});
              }
              return Promise.resolve({ok: true, payload: {session_epoch: 'same-epoch',
                scene_revision: loaded ? 2 : 1, captures: []}});
            },
            live: () => new Promise((resolve, reject) => {rejectOld = reject;}),
            captures: () => new Promise((resolve, reject) => {rejectOld = reject;}),
            loadArchive: async () => {loaded = true; return {ok: true, detail: 'Loaded'};},
          };
          const session = new ReviewSession(api);
          await session.refreshState();
          let oldRequest;
          if (kind === 'live') oldRequest = session.refreshLive();
          if (kind === 'captures') oldRequest = session.refreshCaptures();
          await session.loadArchive('/srv/archive.json');
          rejectOld(new Error('old request connection lost'));
          if (oldRequest) await oldRequest;
          await Promise.resolve();
          assert.equal(session.app.state.pairs[0].id, 9);
          assert.equal(session.app.notice, '', `${kind}: stale failure changed the notice`);
          assert.equal(scheduledRetries.length, 0, `${kind}: stale failure scheduled a retry`);
          session.stopEvents();
        }
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout
