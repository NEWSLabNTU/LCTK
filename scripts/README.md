# Scripts for perception

## Record-data

- "wayside" folders represent different machines.
- Recording data with exectuing **record.sh** in folder, e.g :
```bash=
./wayside1/record.sh 0900 3600

    <to record for 1 hour since 09:00 AM>
```

## Lidar-to-camera

- **00_convert-to-LCTK.sh**
    - Convert the recording data to fit LCTK format.
- You can execute the steps of calibration separately : 
```bash=
./01_convert-to-pcd.sh
./02_convert-to-image.sh
./03_detect-board.sh
./04_detect-aruco.sh
./05_solve-extrinsics.sh
./06_project-points.sh
```
or only exect one script :
```bash=
./lidar_to_camera.sh
```

## Lidar-to-lidar

- Executing **lidar_to_lidar.sh**. e.g :
```bash=
./lidar_to_lidar.sh --config <lidar-config-path>
```