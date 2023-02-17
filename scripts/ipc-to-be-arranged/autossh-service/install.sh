#!/usr/bin/env bash
echo -n "remote port: "
read port
echo $port
mkdir -p $HOME/.config/systemd/user/
cp -v autossh-140.112.28.112@.service $HOME/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable "autossh-140.112.28.112@$port.service"
systemctl --user restart "autossh-140.112.28.112@$port.service"
