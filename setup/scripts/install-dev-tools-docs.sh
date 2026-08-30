#!/usr/bin/env bash
# mdbook and mdbook-mermaid, for building book/.
#
# Requires cargo. The `dev-tools-docs` step declares `needs=["rust"]` so this can no
# longer be reached without a toolchain -- the old combined step declared only
# `system-base`, warned, and silently marked itself done.

set -e

# Pinned per L-09; override to move a pin deliberately.
MDBOOK_VERSION="${MDBOOK_VERSION:-0.4.40}"
MDBOOK_MERMAID_VERSION="${MDBOOK_MERMAID_VERSION:-0.14.0}"

if ! command -v cargo >/dev/null 2>&1; then
    echo "error: cargo not found. Run the 'rust' step first." >&2
    exit 1
fi

echo "Installing mdbook ${MDBOOK_VERSION}..."
if ! command -v mdbook >/dev/null 2>&1; then
    cargo install --locked --version "${MDBOOK_VERSION}" mdbook
fi

echo "Installing mdbook-mermaid ${MDBOOK_MERMAID_VERSION}..."
if ! command -v mdbook-mermaid >/dev/null 2>&1; then
    cargo install --locked --version "${MDBOOK_MERMAID_VERSION}" mdbook-mermaid
fi

# Deliberately NOT running `mdbook-mermaid install`. book/book.toml points
# additional-js at "js/mermaid.min.js" and those assets are tracked in book/js/;
# `mdbook-mermaid install .` writes its own copies to book/ instead, which book.toml
# never loads. The old combined dev-tools step did this and left two stray untracked
# files (2.5 MB) in the working tree.

echo "Documentation tooling installation complete."
