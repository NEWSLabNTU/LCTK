#!/usr/bin/env bash

SESSION_FILE=$(realpath ./session.json5)
COMMAND_FILE="./commands/git-pull.txt"

pdm run -p ../../py-bin/remote-control delegate-command -- \
    --session $SESSION_FILE \
    --command "$(cat $COMMAND_FILE)"
