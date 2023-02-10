# Multi-wayside project

This module consumes two lidar point cloud data, performs calibration board detection on each of them, and shows a point cloud fusion example.  
After execution, it will save two transformation protobuf files including "lidar1_to_lidar2_pose.pb2" and "lidar2_to_lidar1_pose.pb2".  
Point cloud fusing can be done by transforming lidar1 points by "lidar1_to_lidar2_pose.pb2", or by transforming lidar2 points by "lidar2_to_lidar1_pose.pb2".
## Usage

### Run program

- Run `cd wayside-portal/multi-wayside` to change your directory to multi-wayside. 
- Modify the `config.json` to configure the program.
- Run `cargo run --relese` to start the program.

### Program behavior
After running the program, it will start to detect board on first point clode data and show some result image, and then do the same thing on second point cloud data.
- During detection on each of the point cloud data, you should observe the detection result and decide whether to use the detection result.
- You can press `Enter` key to use the result, or press `Esc` key to skip the result and start detection on next frame data.
- The correct and incorrect board detection example is in the directory "examples". Please make sure the board direction is correct (The red line and yellow line).

### Configuration

The configuration is structured as the following.
- `using_the_same_face_of_marker`: Set true if two lidar sensors was observing the same face of calibration board. Set false otherwise. 
- `output_dir`: The absolute path for a directory to save outputs.
- `pcap1_config`: The config for first pcap file.
- `pcap2_config`: The config for second pcap file.

```toml
{
    "using_same_face_of_marker": true, 
    "output_dir": "output",
    "pcap1_config": { /* config for pcap file 1 */ },
    "pcap2_config": { /* config for pcap file 2 */ }
}

```

The config for pcap file is structured as following.
- `sensor`: This field should be set to "vlp16" or "vlp32", indicating which sensor collected the pcap file. 
- `file_path`: The absolute path for a pcap file.
- `automatically_setting_bounding_box`: This field is recommended to be set to false. Set true only if you know the following three fields value and want to set bounding box automatically.  
- `horizontal_distance_between_lidar_and_marker`: This field will be ignored if `automatically_setting_bounding_box` is set to false.
- `vertical_distance_between_lidar_and_marker`: This field will be ignored if `automatically_setting_bounding_box` is set to false.
- `pcap2_configangle_between_horizontal_line_and_x_axis`: This field will be ignored if `automatically_setting_bounding_box` is set to false.
- `detection_config`: The config for board detection algorithm.
```toml
{
	"sensor": "vlp32",
	"file_path" : "/home/newslab-shih/Downloads/2021-01-28 18:01:05.389212222+08:00.pcap",
	"frame_selected" : null,
	"automatically_setting_bounding_box": false,
        "horizontal_distance_between_lidar_and_marker": 11000,
        "vertical_distance_between_lidar_and_marker": 500,
        "angle_between_horizontal_line_and_x_axis": 93,
	"detection_config" : {
		"board_detector": {
			// max number of RANSAC steps
			"plane_ransac_max_iterations": 500,
			// the loss threshold that a point is considered an inlier
			"plane_ransac_inlier_threshold": 0.05,
			// max number of ICP iterations
			"max_icp_iterations": 20000,
			// pose weight is amount of pose change per iteration
			// the ICP terminates if pose weight is blow this threshold several times
			"icp_pose_weight_threshold": 5e-13,
			// the maximum accepted ICP loss
			"icp_rejection_threshold": 1.0,
			// the length of border margin
			"board_width": 1000,    // mm
			// the radius of circle holes
			"hole_radius": 150,     // mm
			// suppose the center of board is (0, 0)
			// the center of hole will be at (+/- shift, +/- shift)
			"hole_center_shift": 200, // mm
		},
		"filters": [
		    {
			// this filter rejects all points outside of this bbox
			"inclusive_range_filter": {
			    "x_min": -2300,
			    "x_max": 300,
			    "y_min": 10000,
			    "y_max": 12000,
			    "z_min": -600,
			    "z_max": 2000
			}
		    },
		    {
			// this filter rejects all points inside of this bbox
			"exclusive_range_filter": {
			    "x_min": -100,
			    "x_max": 100,
			    "y_min": -100,
			    "y_max": 100,
			    "z_min": -100,
			    "z_max": 100
			}
		    },
		    {
			// this filter rejects all points below this intensity
			"intensity_filter": {
			    "min_intensity": 0
			}
		    }
		],
	    }
}
```
##ISSUES
1. error message "error: linking with `cc` failed: exit code: 1" when compiling
   export LIBTORCH_CXX11_ABI=0
