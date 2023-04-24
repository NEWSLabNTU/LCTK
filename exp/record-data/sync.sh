#!/usr/bin/env bash

SOURCE_DIR=$(realpath ./outputs)
TARGET_DIR=$(realpath ./recording)

pdm run -p $(git rev-parse --show-toplevel)/py-bin/remote-control sync -- \
    --session $(realpath session.json5) \
    --source $SOURCE_DIR \
    --target $TARGET_DIR
