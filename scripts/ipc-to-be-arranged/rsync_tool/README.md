# Sync script

This is a script that uses rsync to sync files between the remote server and local computer.

## Prerequisite

Before using this script, please make sure you have registered the public key on the remote server for remote log in. 

For how to setup the key, please refer to this [link](https://blog.gtwang.org/linux/linux-ssh-public-key-authentication/).

## Usage

Please set up the following variables (the _1 and _2 at the end of variable indicates the first and second server to connect to respectively) before running the script:

- `usr_name` : The user name on the remote server.
- `host_ip` : The IP address of the remote server.
- `src_path` : The absolute path to the directory on the remote server you would like to copy data from.
- `dest_path` : The path to the local directory you would like to store the data in. The default value is set to $(pwd), i.e., current directory.

After setting up the variables, you can simply execute the script by:

```sh
bash ./sync.sh
```

or

```sh
./sync.sh
```

## Common Issues

Error: 

```
bash: ./sync.sh: Permission denied
```

Possible Solution:

Please add the permission to execute the file with the following command:

```sh
chmod +x ./sync.sh
```

