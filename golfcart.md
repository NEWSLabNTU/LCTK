In terminal 1, play the rosbag
```bash
ros2 bag play <rosbag_dir> --loop
```

In terminal 2, decompress images
```bash
just republish <left|right>
```

In terminal 3, launch the solver
```bash
just log_level=info solver_mode=manual lidar-camera seyond_<left|right>.yaml
```

In terminal 4, open the solver's control panel
```bash
just extrinsic-solver-controller
```
