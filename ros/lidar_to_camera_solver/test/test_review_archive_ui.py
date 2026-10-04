"""Archive loading through the browser's public API and menu."""

from test_review_session import run_module


def test_archive_api_posts_only_solver_machine_path():
    result = run_module(
        """
        import assert from 'node:assert/strict';
        import { ReviewApi } from './review_api.js';
        const calls = [];
        globalThis.fetch = async (url, options) => {
          calls.push({url, options});
          return {ok: true, json: async () => ({ok: true, detail: 'Loaded'})};
        };
        const api = new ReviewApi('/review/');
        assert.equal((await api.loadArchive('/srv/captures.json')).ok, true);
        assert.equal(calls[0].url, '/review/api/archive/load');
        assert.equal(calls[0].options.method, 'POST');
        assert.deepEqual(JSON.parse(calls[0].options.body), {path: '/srv/captures.json'});
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


MENU_DOM = """
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { Chrome } from './chrome.js';
class Element {
  constructor() { this.listeners = {}; this.children = []; this.textContent = ''; this.disabled = false; }
  addEventListener(kind, callback) { this.listeners[kind] = callback; }
  click() { this.listeners.click?.({stopPropagation() {}}); }
  replaceChildren(...children) { this.children = children; this.textContent = ''; }
  append(child) { this.children.push(child); }
}
const html = readFileSync('./index.html', 'utf8');
const exportButton = new Element();
const loadButton = new Element();
const autowareButton = new Element();
const notice = new Element();
const diff = new Element();
const elements = new Map([
  ['#exportArchive', exportButton], ['#loadArchive', loadButton],
  ['#exportAutoware', autowareButton], ['#action-notice', notice], ['#autoware', diff],
]);
const root = {
  querySelector: (selector) => elements.get(selector) || null,
  querySelectorAll: (selector) => selector === '#menu .item' ? [exportButton, loadButton, autowareButton] : [],
};
globalThis.document = {createElement: () => new Element()};
const app = {state: {pairs: [], export: {archive_path: '/srv/default.json', autoware_ready: true}},
  layers: {}, clouds: new Map(), preview: {status: 'idle'}, notice: ''};
const tick = () => new Promise((resolve) => setTimeout(resolve, 0));
"""


def test_load_menu_prompts_confirms_and_blocks_duplicates_without_moving_exports():
    result = run_module(
        MENU_DOM
        + r"""
        assert.match(html, /id="exportArchive"[^>]*>Export archive[^<]*<\/button>\s*<button[^>]*id="loadArchive"/);
        let answer = null;
        let confirmed = false;
        let confirmations = 0;
        let prompts = 0;
        let finishLoad;
        const loads = [];
        const exports = [];
        let previews = 0;
        globalThis.prompt = (text, initial) => {
          prompts += 1;
          assert.match(text, /solver/i);
          assert.equal(initial, '/srv/default.json');
          return answer;
        };
        globalThis.confirm = (text) => {
          confirmations += 1;
          assert.match(text, /replace/i);
          return confirmed;
        };
        const chrome = new Chrome(root, {
          onLoadArchive: (path) => { loads.push(path); return new Promise((resolve) => {finishLoad = resolve;}); },
          onExportArchive: (path) => {exports.push(path); return {ok: true, detail: 'Saved'};},
          onAutowarePreview: () => { previews += 1; return {ok: true, detail: 'Preview'}; },
        });
        chrome.render(app);
        loadButton.click();
        answer = '   ';
        loadButton.click();
        assert.equal(loads.length, 0);
        assert.equal(confirmations, 0);
        app.state.pairs = [{id: 3}];
        answer = ' /srv/replacement.json ';
        loadButton.click();
        assert.equal(loads.length, 0);
        confirmed = true;
        loadButton.click();
        assert.deepEqual(loads, ['/srv/replacement.json']);
        assert.equal(loadButton.disabled, true);
        const promptsBeforeDuplicate = prompts;
        loadButton.click();
        assert.equal(prompts, promptsBeforeDuplicate);
        finishLoad({ok: true, detail: 'Loaded'});
        await tick();
        assert.equal(loadButton.disabled, false);
        app.state.pairs = [];
        const confirmationsBeforeEmpty = confirmations;
        loadButton.click();
        assert.equal(confirmations, confirmationsBeforeEmpty);
        finishLoad({ok: false, detail: 'Wrong Target Identity'});
        await tick();
        assert.match(notice.textContent, /Wrong Target Identity/);
        exportButton.click();
        autowareButton.click();
        assert.deepEqual(exports, ['/srv/default.json']);
        assert.equal(previews, 1);
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout


def test_loaded_archive_clears_visible_autoware_confirmation_even_when_refresh_fails():
    result = run_module(
        MENU_DOM
        + """
        let loadOk = false;
        const chrome = new Chrome(root, {
          onAutowarePreview: async () => ({ok: true, entry: {old_transform: true}}),
          onAutowareWrite: async () => ({ok: true}),
          onLoadArchive: async () => loadOk
            ? {ok: true, detail: 'Archive loaded, but review refresh failed. Retrying.'}
            : {ok: false, detail: 'Wrong Target Identity'},
        });
        globalThis.prompt = () => '/srv/archive.json';
        chrome.render(app);
        autowareButton.click();
        await tick();
        assert.equal(diff.children.length, 2);
        assert.equal(diff.children[1].textContent, 'Confirm write');
        loadButton.click();
        await tick();
        assert.equal(diff.children.length, 2, 'failed loads retain the current preview');
        loadOk = true;
        loadButton.click();
        await tick();
        assert.equal(diff.children.length, 0, 'committed loads remove stale confirmation');
        assert.match(notice.textContent, /Archive loaded.*refresh failed/);
        chrome.render(app);
        assert.equal(diff.children.length, 0, 'render must not recreate stale confirmation');
        """
    )
    assert result.returncode == 0, result.stderr or result.stdout
