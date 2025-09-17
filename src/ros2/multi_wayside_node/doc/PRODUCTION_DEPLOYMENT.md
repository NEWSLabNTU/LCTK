# Multi-Wayside Node Production Deployment Guide

## Overview

This guide provides best practices and recommendations for deploying the multi_wayside_node in production environments. It covers system requirements, configuration optimization, monitoring, and maintenance procedures.

## System Requirements

### Hardware Requirements

#### Minimum Configuration
- CPU: 4 cores @ 2.4 GHz
- RAM: 8 GB
- Storage: 10 GB available
- Network: Gigabit Ethernet for sensor data

#### Recommended Configuration
- CPU: 8+ cores @ 3.0 GHz
- RAM: 16 GB
- Storage: 50 GB available (for logging and data retention)
- Network: 10 Gigabit Ethernet for high-frequency sensor data

### Software Requirements
- Ubuntu 20.04 LTS or 22.04 LTS
- ROS 2 Humble or later
- Real-time kernel (recommended for < 10ms latency requirements)
- NTP time synchronization configured

## Pre-Deployment Checklist

### 1. System Preparation
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    git \
    curl \
    chrony

# Configure time synchronization
sudo systemctl enable chrony
sudo systemctl start chrony
```

### 2. Network Configuration
```bash
# Set up static IP for sensor network
sudo nano /etc/netplan/01-netcfg.yaml

# Example configuration:
# network:
#   version: 2
#   ethernets:
#     eth0:
#       addresses: [192.168.1.100/24]
#       gateway4: 192.168.1.1
#       nameservers:
#         addresses: [8.8.8.8, 8.8.4.4]

sudo netplan apply
```

### 3. Resource Limits
```bash
# Increase file descriptor limits
echo "* soft nofile 65536" | sudo tee -a /etc/security/limits.conf
echo "* hard nofile 65536" | sudo tee -a /etc/security/limits.conf

# Configure kernel parameters for real-time performance
echo "net.core.rmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
echo "net.core.wmem_max = 134217728" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## Production Configuration

### 1. Optimized Parameters

Create `config/production_params.yaml`:
```yaml
# Production-optimized parameters
multi_wayside_node:
  ros__parameters:
    # Core configuration
    board_config_file: "/opt/lctk/config/hollow_board.yaml"
    detector_config_file: "/opt/lctk/config/detector.yaml"
    aruco_pattern_file: "/opt/lctk/config/aruco_pattern.json5"

    # Automatic calibration
    auto_calibrate: true
    min_detections_for_calibration: 10  # Higher threshold for production
    calibration_timeout_seconds: 60
    quality_threshold: 0.8  # Stricter quality requirement

    # Performance tuning
    max_queue_size: 200
    sync_tolerance_ms: 50  # Tighter synchronization

    # ROI configuration (site-specific)
    roi_box_position_x: 3.0
    roi_box_position_y: 0.0
    roi_box_position_z: 0.5
    roi_box_size_x: 6.0
    roi_box_size_y: 6.0
    roi_box_size_z: 3.0

    # Filtering
    min_range: 1.0  # Filter very close points
    max_range: 30.0  # Limit to relevant range

    # Logging
    log_level: "info"  # Reduce verbosity in production
```

### 2. Launch Configuration

Create `launch/production.launch.xml`:
```xml
<launch>
    <!-- Production deployment configuration -->
    <arg name="config_file" default="/opt/lctk/config/production_params.yaml"/>
    <arg name="enable_diagnostics" default="true"/>
    <arg name="enable_monitoring" default="true"/>

    <!-- Multi-wayside node -->
    <node pkg="multi_wayside_node" exec="multi_wayside_node" name="multi_wayside_production"
          output="screen" respawn="true" respawn_delay="5">
        <param from="$(var config_file)"/>

        <!-- Topic remapping for production -->
        <remap from="/lidar1/points" to="/sensors/lidar1/points"/>
        <remap from="/lidar2/points" to="/sensors/lidar2/points"/>
    </node>

    <!-- Diagnostics aggregator -->
    <node pkg="diagnostic_aggregator" exec="aggregator_node" name="diagnostic_aggregator" if="$(var enable_diagnostics)">
        <param from="/opt/lctk/config/diagnostics.yaml"/>
    </node>

    <!-- Monitoring node -->
    <node pkg="multi_wayside_node" exec="health_monitor.py" name="health_monitor" if="$(var enable_monitoring)">
        <param name="alert_email" value="ops@example.com"/>
        <param name="check_interval" value="30.0"/>
    </node>
</launch>
```

## Deployment Procedures

### 1. Installation
```bash
# Create installation directory
sudo mkdir -p /opt/lctk
sudo chown $USER:$USER /opt/lctk

# Copy files
cp -r ~/lctk_workspace/install/multi_wayside_node/* /opt/lctk/
cp -r config /opt/lctk/

# Create systemd service
sudo tee /etc/systemd/system/multi-wayside.service > /dev/null <<EOF
[Unit]
Description=Multi-Wayside LiDAR Calibration Service
After=network.target

[Service]
Type=simple
User=$USER
Environment="ROS_DOMAIN_ID=42"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /opt/lctk/setup.bash && ros2 launch multi_wayside_node production.launch.xml'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable multi-wayside.service
sudo systemctl start multi-wayside.service
```

### 2. Verification
```bash
# Check service status
sudo systemctl status multi-wayside.service

# Monitor logs
sudo journalctl -u multi-wayside.service -f

# Verify topics
ros2 topic list | grep -E "(lidar|board_detection|calibration)"

# Check calibration output
ros2 topic echo /calibration_transform
```

## Monitoring and Alerting

### 1. Health Monitoring Script

Create `/opt/lctk/scripts/health_monitor.py`:
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import smtplib
from email.mime.text import MIMEText
from datetime import datetime, timedelta

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.declare_parameter('alert_email', '')
        self.declare_parameter('check_interval', 30.0)

        self.last_detection = {
            'lidar1': None,
            'lidar2': None
        }
        self.last_calibration = None

        # Create timer for periodic checks
        interval = self.get_parameter('check_interval').value
        self.timer = self.create_timer(interval, self.check_health)

    def check_health(self):
        # Check detection freshness
        now = datetime.now()
        for lidar, last_time in self.last_detection.items():
            if last_time and (now - last_time) > timedelta(minutes=5):
                self.send_alert(f"No detections from {lidar} for 5 minutes")

        # Check calibration status
        if self.last_calibration and (now - self.last_calibration) > timedelta(hours=1):
            self.send_alert("No calibration updates for 1 hour")

    def send_alert(self, message):
        email = self.get_parameter('alert_email').value
        if email:
            # Send email alert
            self.get_logger().error(f"ALERT: {message}")
```

### 2. Performance Monitoring
```bash
# Run performance profiler periodically
0 */4 * * * /opt/lctk/scripts/profile_performance.py --duration 300 --output-dir /var/log/lctk/performance

# Monitor system resources
*/5 * * * * /opt/lctk/scripts/log_system_stats.sh >> /var/log/lctk/system_stats.log
```

### 3. Log Rotation
```bash
# Create logrotate configuration
sudo tee /etc/logrotate.d/multi-wayside > /dev/null <<EOF
/var/log/lctk/*.log {
    daily
    rotate 14
    compress
    delaycompress
    missingok
    notifempty
    create 0644 $USER $USER
}
EOF
```

## Calibration Best Practices

### 1. Environmental Setup
- **Board Placement**: Position calibration board 3-5 meters from sensors
- **Lighting**: Ensure consistent ambient lighting
- **Stability**: Mount sensors rigidly to minimize vibration
- **Clear Path**: Remove obstacles between sensors and board

### 2. Operational Procedures
```bash
# Daily calibration check
ros2 service call /trigger_calibration std_srvs/srv/Trigger

# Weekly full validation
ros2 launch multi_wayside_node validation_test.launch.py

# Monthly performance review
python3 /opt/lctk/scripts/analyze_monthly_performance.py
```

### 3. Quality Thresholds
- **Excellent**: Confidence > 0.9, use immediately
- **Good**: Confidence 0.8-0.9, verify visually
- **Marginal**: Confidence 0.7-0.8, recalibrate
- **Poor**: Confidence < 0.7, investigate issues

## Troubleshooting

### Common Issues and Solutions

#### 1. No Detections
```bash
# Check point cloud data
ros2 topic hz /sensors/lidar1/points
ros2 topic echo /sensors/lidar1/points --no-arr | head -20

# Verify ROI settings
ros2 service call /get_roi_bounds multi_wayside_node/srv/GetROIBounds "{lidar_id: 1}"

# Increase ROI size
ros2 service call /set_roi_bounds multi_wayside_node/srv/SetROIBounds \
  "{lidar_id: 1, center_x: 3.0, center_y: 0.0, center_z: 0.0, size_x: 8.0, size_y: 8.0, size_z: 4.0}"
```

#### 2. Poor Calibration Quality
```bash
# Reset and retry
ros2 service call /reset_calibration std_srvs/srv/Trigger

# Increase detection requirements
ros2 param set /multi_wayside_production min_detections_for_calibration 15

# Check synchronization
ros2 param set /multi_wayside_production sync_tolerance_ms 100
```

#### 3. High CPU Usage
```bash
# Reduce point cloud processing
ros2 param set /multi_wayside_production min_range 2.0
ros2 param set /multi_wayside_production max_range 20.0

# Limit queue sizes
ros2 param set /multi_wayside_production max_queue_size 50
```

## Maintenance Schedule

### Daily
- Monitor service status and logs
- Verify calibration freshness
- Check for alerts

### Weekly
- Run validation tests
- Review performance metrics
- Clean sensor lenses

### Monthly
- Full system performance analysis
- Update software if needed
- Backup configuration and calibration data

### Quarterly
- Hardware inspection
- Network infrastructure review
- Disaster recovery drill

## Backup and Recovery

### 1. Configuration Backup
```bash
# Automated daily backup
0 2 * * * tar -czf /backup/lctk_config_$(date +\%Y\%m\%d).tar.gz /opt/lctk/config

# Backup calibration results
*/30 * * * * cp /opt/lctk/calibration_transform.yaml /backup/calibration_latest.yaml
```

### 2. Recovery Procedure
```bash
# Stop service
sudo systemctl stop multi-wayside.service

# Restore configuration
tar -xzf /backup/lctk_config_YYYYMMDD.tar.gz -C /

# Restore calibration
cp /backup/calibration_latest.yaml /opt/lctk/

# Restart service
sudo systemctl start multi-wayside.service
```

## Security Considerations

### 1. Network Security
- Use dedicated VLAN for sensor network
- Implement firewall rules for ROS 2 ports
- Enable DDS security if available

### 2. Access Control
```bash
# Restrict file permissions
chmod 750 /opt/lctk
chmod 640 /opt/lctk/config/*

# Create dedicated user
sudo useradd -r -s /bin/false lctk_service
sudo chown -R lctk_service:lctk_service /opt/lctk
```

### 3. Audit Logging
```bash
# Enable audit logging for calibration changes
auditctl -w /opt/lctk/config -p wa -k lctk_config_changes
```

## Performance Tuning

### 1. DDS Configuration

Create `/opt/lctk/cyclonedds.xml`:
```xml
<CycloneDDS>
  <Domain>
    <General>
      <NetworkInterfaceAddress>192.168.1.100</NetworkInterfaceAddress>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
  <Tracing>
    <Verbosity>warning</Verbosity>
  </Tracing>
</CycloneDDS>
```

### 2. CPU Affinity
```bash
# Pin multi-wayside process to specific cores
sudo apt install schedtool
schedtool -a 2,3,4,5 -e ros2 launch multi_wayside_node production.launch.xml
```

## Conclusion

This production deployment guide provides a comprehensive framework for deploying and maintaining the multi_wayside_node in production environments. Regular monitoring, proper configuration, and adherence to maintenance schedules will ensure reliable operation and high-quality calibration results.

For additional support, refer to:
- [API Reference](API_REFERENCE.md)
- [User Guide](USER_GUIDE.md)
- [Troubleshooting Guide](../README.md#troubleshooting)