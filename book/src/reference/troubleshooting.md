# Troubleshooting

This guide helps resolve common issues encountered when using LCTK.

## Build Issues

### OpenCV Binding Generation Failure

**Error**: `fatal error: 'memory' file not found`

**Cause**: Missing C++ development headers
**Solution**:
```bash
sudo apt-get install libstdc++-12-dev libclang-dev
export OPENCV_PKGCONFIG_NAME=opencv4
export OpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
make build
```

### SFCGAL Library Missing

**Error**: `SFCGAL/capi/sfcgal_c.h: No such file or directory`

**Cause**: Missing SFCGAL development libraries
**Solution**:
```bash
# Install SFCGAL
sudo apt-get install libsfcgal-dev

# Or exclude packages that need SFCGAL
colcon build --packages-skip multi_wayside multi_wayside_node extrinsic_solver
```

### Colcon Build Aborts

**Error**: One package failure causes all subsequent packages to abort

**Cause**: Colcon's dependency resolution stops on first failure
**Solution**:
```bash
# Fix the failing package's dependencies first
# Check the specific error in the build log

# Or build packages individually
colcon build --packages-select <working_package>

# Build with continue-on-error flag
colcon build --continue-on-error
```

### JSON Parsing Errors in colcon-cargo

**Error**: `JSONDecodeError: Expecting value: line 1 column 1`

**Cause**: Cargo metadata output contains patch warnings
**Solution**: Fixed by modifying colcon-cargo to use `--quiet` flag
```bash
# If still encountering this issue, ensure you have the latest colcon-cargo
pip install --upgrade colcon-cargo
```

## Runtime Issues

### No ArUco Detections

**Symptoms**: `/calibration/aruco_locator/aruco_detections` topic active but no successful detections

**Diagnostic Steps**:
```bash
# Check camera info topic
ros2 topic echo /sensing/camera/front_center/camera_info

# Verify image topic
ros2 topic echo /sensing/camera/front_center/image_raw

# Check ArUco locator logs
ros2 node info /calibration/aruco_locator
```

**Common Causes & Solutions**:

1. **Empty camera_info**: Camera calibration data missing
   ```bash
   # Verify camera_info contains non-zero values
   # Check camera_info_url parameter in launch file
   ```

2. **Poor lighting**: ArUco markers not clearly visible
   ```bash
   # Adjust camera exposure settings
   # Improve lighting conditions
   # Clean marker surfaces
   ```

3. **Incorrect marker configuration**:
   ```bash
   # Verify marker IDs match physical markers
   # Check marker size configuration
   # Validate dictionary type
   ```

### Board Detection Failures

**Symptoms**: No board detections in point cloud data

**Diagnostic Steps**:
```bash
# Visualize point cloud
ros2 run rviz2 rviz2
# Add PointCloud2 display, subscribe to lidar topic

# Check board locator parameters
ros2 param list /calibration/calibration_board_locator
```

**Common Causes & Solutions**:

1. **Insufficient point cloud density**:
   ```bash
   # Reduce voxel grid filter size
   # Increase LiDAR scan resolution
   # Move calibration board closer
   ```

2. **Board not in field of view**:
   ```bash
   # Check LiDAR range and angle limits
   # Verify board position and orientation
   # Adjust detection ROI parameters
   ```

3. **Plane detection threshold issues**:
   ```bash
   # Adjust max_plane_distance parameter
   # Modify min_plane_points threshold
   # Check RANSAC iteration count
   ```

### Synchronization Problems

**Symptoms**: Detections from ArUco and board locators are not synchronized

**Diagnostic Steps**:
```bash
# Check synchronizer topic
ros2 topic echo /calibration/synchronizer/synchronized_detections

# Verify timestamp alignment
ros2 topic echo --field header.stamp /calibration/aruco_locator/aruco_detections
```

**Solutions**:
```bash
# Adjust synchronization tolerance
ros2 param set /calibration/synchronizer sync_window_ms 100

# Check system time synchronization
chrony sources -v

# Verify sensor timestamp accuracy
```

### Poor Calibration Accuracy

**Symptoms**: High reprojection errors, inconsistent results

**Diagnostic Steps**:
```bash
# Check calibration quality metrics
ros2 topic echo /calibration/extrinsic_solver/calibration_quality

# Verify detection consistency
# Monitor convergence indicators
```

**Solutions**:
1. **Increase detection count**: Collect more calibration frames
2. **Improve target visibility**: Better lighting, cleaner markers
3. **Check camera calibration**: Verify intrinsic parameters
4. **Validate target geometry**: Ensure accurate physical measurements

## Performance Issues

### Slow Detection Times

**Symptoms**: Detection processing takes >1 second per frame

**Diagnostic Solutions**:
```bash
# Enable GPU acceleration (if available)
export CUDA_VISIBLE_DEVICES=0

# Reduce point cloud size
# Adjust voxel filter parameters
# Optimize detection ROI

# Use performance monitoring
htop  # Monitor CPU usage
nvidia-smi  # Monitor GPU usage
```

### High Memory Usage

**Symptoms**: System runs out of memory during processing

**Solutions**:
```bash
# Reduce buffer sizes in synchronizer
# Enable point cloud downsampling
# Limit concurrent processing threads

# Monitor memory usage
free -h
ps aux --sort=-%mem | head
```

### ROS 2 Communication Issues

**Symptoms**: Topics not visible, nodes not communicating

**Diagnostic Steps**:
```bash
# Check ROS 2 daemon
ros2 daemon status
ros2 daemon stop && ros2 daemon start

# Verify node connectivity
ros2 node list
ros2 topic list

# Check network configuration
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION
```

## Hardware-Specific Issues

### Camera Issues

**No image data**:
```bash
# Check USB connection and permissions
lsusb
ls -la /dev/video*

# Test camera directly
v4l2-ctl --list-devices
```

**Poor image quality**:
```bash
# Adjust camera parameters
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=128
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=32
```

### LiDAR Issues

**No point cloud data**:
```bash
# Check Ethernet connection
ip addr show
ping <lidar_ip>

# Verify PCAP file integrity
tcpdump -r /path/to/lidar.pcap | head
```

**Point cloud quality issues**:
```bash
# Check LiDAR settings
# Verify mounting stability
# Clean LiDAR sensor windows
```

## Debugging Tools

### Log Analysis
```bash
# ROS 2 logs
ros2 log info <node_name>
tail -f ~/.ros/log/<session_id>/<node>-*.log

# System logs
journalctl -u <service_name> -f
dmesg | tail
```

### Visualization Tools
```bash
# RViz for 3D visualization
rviz2 -d config/calibration_debug.rviz

# rqt for topic monitoring
rqt_graph  # Node graph
rqt_topic  # Topic monitor
rqt_plot   # Real-time plotting
```

### Performance Profiling
```bash
# CPU profiling
perf record -g ros2 run <package> <node>
perf report

# Memory profiling
valgrind --tool=memcheck ros2 run <package> <node>

# ROS 2 performance analysis
ros2 topic hz <topic_name>
ros2 topic bw <topic_name>
```

## Getting Help

### Information to Gather
When reporting issues, include:
- LCTK version and commit hash
- ROS 2 distribution and version
- Operating system and kernel version
- Hardware specifications
- Complete error messages and logs
- Steps to reproduce the issue

### Support Channels
- GitHub Issues: Bug reports and feature requests
- GitHub Discussions: General questions and community help
- Documentation: Check this guide and API references first

### Advanced Debugging
For complex issues:
1. Enable debug mode in launch files
2. Increase logging verbosity
3. Use GDB for crash analysis
4. Capture network traffic for communication issues
5. Profile performance bottlenecks