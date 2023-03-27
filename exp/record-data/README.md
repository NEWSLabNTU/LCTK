# Data Recording


## Launch the connection

Configure the config file of connecting session under the folder _./config/session_ and make a symbolic link to what you want to use.
For instance,

```bash
ln -snf ./config/session/template.json5 session.json5
```

Then run the following script to setup the connections. The connections will be served in several panes in Tmux.

```bash
./launch.sh
```

## Start recording

1. The default camera mapping file is _./config/camera-id.json5_.
2. Confgirue the config file of recording recipes under the folder _./config/recipes_.
3. Run the following script to delegate the actions.

```bash
./delegate-recipe.sh [./config/recipes/template.json5]
```


The recording format looks like below

```bash
{O}/{N}/{T}/wayside{W}/video/camera{C}.mp4
{O}/{N}/{T}/wayside{W}/video/camera{C}.txt
{O}/{N}/{T}/wayside{W}/pcd/lidar1.pcap
```

where

```bash
O = output directory
N = recording recipe name
T = sample timestamp in RFC3339 from host
W = wayside sensor index: 1~3
C = camera index: 1~3
```

## Local Backup

In practice, we may move the recording data to its local storage on each wayside sensor.
Then we can use the following script to achieve it.

_./commands/backup.txt_
```txt
rsync -Arxv ./outputs/ ./backup
```

## Data Collection

The following script follows the the session config _session.json5_ and pull the data back via rsync.
User may specify the `SOURCE_DIR` and `TARGET_DIR` to adjust the data synchronization path.

```bash
./sync.sh
```
