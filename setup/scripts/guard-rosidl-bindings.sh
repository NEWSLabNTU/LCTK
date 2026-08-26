#!/usr/bin/env bash
# Keep colcon-cargo-ros2's one-shot rosidl binding cache aligned with the
# interface sources.  This script intentionally accepts only the one generated
# wrapper layout that it may delete.

set -euo pipefail

usage() {
    echo "usage: $0 --check|--record [cargo-config]" >&2
    exit 2
}

[[ $# -ge 1 && $# -le 2 ]] || usage
mode=$1
[[ "$mode" == --check || "$mode" == --record ]] || usage

repo_root=${LCTK_REPO_ROOT:-"$(pwd -P)"}
repo_root=$(realpath -e -- "$repo_root")
config=${2:-"$repo_root/.cargo/config.toml"}
[[ -f "$config" ]] || exit 0

build_root=$(realpath -m -- "$repo_root/build")
manifest_dir="$build_root/.colcon/lctk-interface-manifests"

remove_lock() {
    rm -f -- "$build_root/.colcon/bindgen.lock"
}

source_manifest() {
    local source_dir=$1
    (
        cd "$source_dir"
        find . -type f \( -path './msg/*' -o -path './srv/*' -o -path './action/*' \) -print0 |
            sort -z | xargs -0 -r sha256sum
    )
}

while IFS= read -r path; do
    # A missing patch target is enough to invalidate bindgen, irrespective of
    # whether it is one of our interface wrappers.
    if [[ ! -f "$repo_root/$path/Cargo.toml" ]]; then
        echo "bindgen output missing ($path); removing stale bindgen.lock"
        remove_lock
        continue
    fi

    # Only a literal build/<pkg>/rosidl_cargo/<same-pkg> entry is eligible for
    # wrapper removal.  In particular, do not normalise and then trust ./.. .
    if [[ ! "$path" =~ ^build/([^/]+)/rosidl_cargo/([^/]+)$ ]]; then
        continue
    fi
    package=${BASH_REMATCH[1]}
    wrapper_package=${BASH_REMATCH[2]}
    if [[ "$package" == . || "$package" == .. || "$wrapper_package" == . || "$wrapper_package" == .. || "$package" != "$wrapper_package" ]]; then
        continue
    fi

    expected=$(realpath -m -- "$build_root/$package/rosidl_cargo/$package")
    canonical_wrapper=$(realpath -m -- "$repo_root/$path")
    # Both equality and containment checks make an accidental config rewrite
    # fail closed, before the one destructive operation below.
    if [[ "$canonical_wrapper" != "$expected" || "$expected" != "$build_root"/* ]]; then
        continue
    fi

    interface_dir="$repo_root/ros/$package"
    [[ -d "$interface_dir" ]] || continue
    manifest="$manifest_dir/$package.sha256"
    current=$(source_manifest "$interface_dir")

    if [[ "$mode" == --record ]]; then
        mkdir -p -- "$manifest_dir"
        tmp=$(mktemp "$manifest.XXXXXX")
        printf '%s\n' "$current" >"$tmp"
        mv -f -- "$tmp" "$manifest"
    elif [[ ! -f "$manifest" ]]; then
        # On the first build after introducing this guard, there is no baseline
        # from which a past deletion can be inferred.  Regenerate once, then
        # persist the manifest after the successful build below.
        echo "interface source manifest missing ($package); regenerating wrapper"
        rm -rf -- "$expected"
        remove_lock
    elif ! cmp -s "$manifest" <(printf '%s\n' "$current"); then
        echo "interface source manifest changed ($package); regenerating wrapper"
        rm -rf -- "$expected"
        remove_lock
    fi
done < <(grep -oP 'path = "\K[^"]+' "$config")
