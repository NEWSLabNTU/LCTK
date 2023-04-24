#!/usr/bin/env bash

SESSION_FILE=$(realpath ./session.json5)

pdm run -p $(git rev-parse --show-toplevel)/py-bin/remote-control launch -- -s $SESSION_FILE -w $(pwd)
