#!/usr/bin/env bash

SESSION_FILE=$(realpath ./session.json5)
RECIPE_FILE=$(realpath ${1:-./config/recipes/template.json5})
ROOT_DIR=$(git rev-parse --show-toplevel)

OUTPUT_DIR=$(realpath ./outputs)
CAMERA_MAPPING=$(realpath ./config/camera-id.json5)

pdm run -p $ROOT_DIR/py-bin/remote-control delegate-recipe -- \
    --session $SESSION_FILE \
    --recipe $RECIPE_FILE \
    --output-dir $OUTPUT_DIR \
    --mapping $CAMERA_MAPPING
