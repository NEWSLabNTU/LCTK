#!/usr/bin/env bash
# Safe regression test for guard-rosidl-bindings.sh.  It uses a disposable
# fixture under the project tmp directory and never references this checkout's
# build directory.

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
fixture=$(mktemp -d "$repo_root/tmp/rosidl-guard-test.XXXXXX")
trap 'rm -rf -- "$fixture"' EXIT

mkdir -p "$fixture/.cargo" "$fixture/ros/demo/msg" \
    "$fixture/build/demo/rosidl_cargo/demo" "$fixture/build/.colcon"
printf 'int32 value\n' >"$fixture/ros/demo/msg/Value.msg"
touch "$fixture/build/demo/rosidl_cargo/demo/Cargo.toml" \
    "$fixture/build/.colcon/bindgen.lock"
printf '[patch.crates-io]\npath = "build/demo/rosidl_cargo/demo"\n' >"$fixture/.cargo/config.toml"

reset_wrapper() {
    mkdir -p "$fixture/build/demo/rosidl_cargo/demo" "$fixture/build/.colcon"
    touch "$fixture/build/demo/rosidl_cargo/demo/Cargo.toml" \
        "$fixture/build/.colcon/bindgen.lock"
}

# A pre-existing wrapper without a manifest is regenerated once, establishing a
# baseline that also makes a deletion before this guard's first run safe.
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ ! -e "$fixture/build/demo/rosidl_cargo/demo" ]]
[[ ! -e "$fixture/build/.colcon/bindgen.lock" ]]
reset_wrapper
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --record "$fixture/.cargo/config.toml"

# Editing an existing interface invalidates the wrapper.
printf 'int64 value\n' >"$fixture/ros/demo/msg/Value.msg"
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ ! -e "$fixture/build/demo/rosidl_cargo/demo" ]]
[[ ! -e "$fixture/build/.colcon/bindgen.lock" ]]

# Adding an interface invalidates the wrapper.
reset_wrapper
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --record "$fixture/.cargo/config.toml"
printf 'string value\n' >"$fixture/ros/demo/msg/Added.msg"
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ ! -e "$fixture/build/demo/rosidl_cargo/demo" ]]
[[ ! -e "$fixture/build/.colcon/bindgen.lock" ]]

# Deleting an interface invalidates the wrapper.
reset_wrapper
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --record "$fixture/.cargo/config.toml"
rm "$fixture/ros/demo/msg/Added.msg"
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ ! -e "$fixture/build/demo/rosidl_cargo/demo" ]]
[[ ! -e "$fixture/build/.colcon/bindgen.lock" ]]

# A malformed package path must not be normalised into a deletion target.
mkdir -p "$fixture/build/demo/rosidl_cargo/not-demo"
touch "$fixture/build/demo/rosidl_cargo/not-demo/Cargo.toml" "$fixture/build/keep"
printf '[patch.crates-io]\npath = "build/demo/rosidl_cargo/not-demo"\n' >"$fixture/.cargo/config.toml"
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ -e "$fixture/build/demo/rosidl_cargo/not-demo/Cargo.toml" ]]
[[ -e "$fixture/build/keep" ]]

# Dot segments are also rejected before canonicalisation can turn them into a
# plausible-looking deletion target.
mkdir -p "$fixture/build/rosidl_cargo"
touch "$fixture/build/rosidl_cargo/Cargo.toml"
printf '[patch.crates-io]\npath = "build/./rosidl_cargo/."\n' >"$fixture/.cargo/config.toml"
LCTK_REPO_ROOT="$fixture" "$repo_root/setup/scripts/guard-rosidl-bindings.sh" --check "$fixture/.cargo/config.toml"
[[ -e "$fixture/build/rosidl_cargo/Cargo.toml" ]]

echo "guard-rosidl-bindings: ok"
