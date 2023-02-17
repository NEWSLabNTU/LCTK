#!/usr/bin/env bash
rm -f $HOME/.config/systemd/user/autossh-140.112.28.112@*.service
systemctl --user daemon-reload
