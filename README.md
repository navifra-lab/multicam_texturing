# LIO-SAM-COLOR

**A modified version of LIO-SAM to create colorful maps.**

<p align='center'>
    <img src="./config/doc/demo.png" alt="drawing" width="800"/>
</p>

## System architecture

<p align='center'>
    <img src="./config/doc/system.png" alt="drawing" width="800"/>
</p>

Unlike the original [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM), it takes a dynamic number of camera data as input, and it creates colorful maps.

> **_NOTE:_** **Please be sure you can run original LIO-SAM before create colorful map.**

## Required Data

To create color maps with LIO-SAM, there is need for ROS2 bag file and ROS2 bag file should contain these topics:

- `sensor_msgs/msg/PointCloud2`
- `sensor_msgs/msg/Imu`
- `sensor_msgs/msg/Odometry` (optional)
- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CameraInfo`

## Usage

### 1. Clone and build the package

```bash
git clone https://github.com/leo-drive/LIO-SAM-COLOR.git
cd LIO-SAM

colcon build --symlink-install
```

### 2. Configure the parameters

```bash
pointCloudTopic: "/points" # required: your lidar topic
imuTopic: "/imu/data" # required: your imu topic
gpsTopic: "odometry/gpsz" # optional: your odometry topic

useImuHeadingInitialization: true # set true if you use odometry
useGpsElevation: true # set true if you use odometry

# select your lidar sensor from supported types. If your lidar not sport, select velodyne and change other lidar parameters with using your lidar features.
sensor: velodyne
N_SCAN: 32
Horizon_SCAN: 1800

# set your lidar-imu calibration
extrinsicRot: [1.0,  0.0,  0.0,
                  0.0,  1.0,  0.0,
                  0.0,  0.0, 1.0 ]
extrinsicRPY: [ 1.0,  0.0,  0.0,
                  0.0,  1.0,  0.0,
                  0.0,  0.0,  1.0 ]
                  
# Example Parameters
#
# Example Image Topics: "/sensing/camera/camera0/image_raw", "/sensing/camera/camera1/image_raw", "/sensing/camera/camera2/image_raw"
# Example Camera Info Topics: "/sensing/camera/camera0/camera_info", "/sensing/camera/camera1/camera_info", "/sensing/camera/camera2/camera_info"
#
# For the above examples, parameters should be set as follows.

cameraTopics: ["/sensing/camera/camera0/", "/sensing/camera/camera1/", "/sensing/camera/camera2/"]
imageTopicLastName: "image_raw"
cameraInfoTopicLastName: "camera_info"
```

### 3. Run the package

> **_WARNING:_** **Please be sure you publish the transform between lidar and each camera.**
> ```bash
> ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw lidar_frame camera_frame 100
> ```

```bash
source install/setup.bash
ros2 launch lio_sam run.launch.py

ros2 bag play <your_rosbag_file>
```

### 4. Save the map

When mapping process is finished, run following command and save slam map to specified path. Be sure you are in LIO-SAM directory.

```bash
source install/setup.bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /Downloads/service_LOAM}"
```