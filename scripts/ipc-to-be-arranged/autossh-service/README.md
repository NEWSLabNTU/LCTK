# SSH Tunneling Service

The script installs a systemd service that establishes a SSH tunnel from remote server to the localhost. The remote host is set to 140.112.28.112 by default.

Run `./install.sh` to install the service. It asks your desired port opened on remote, for example, 22222.

```sh
./install.sh
remote port: 22222
```

To connect to the tunnel on any other machine, run `ssh` command like this.

```sh
ssh -p 22222 newslab@140.112.28.112
```
