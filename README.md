# giraffe_v2
A low-cost embodied-ai and ros2 enabled mobile manipulator using Giraffe arm.

Start instructions:

1. Turn on xavier

2. Attach to Docker env:
```bash
ssh arpit@192.168.1.23 # or ssh arpit@192.168.55.1 via usb
    # pass: arpitmtv
app
./attach_giraffe_docker.sh
# pass: arpitmtv
cd app/giraffe_v2/giraffe_v2_ws/
```

3. Start all nodes:

```bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py # RPLidar A1
ros2 launch realsense2_camera rs_launch.py # realsense camera
python3 /app/giraffe_v2/giraffe_v2_ws/src/teleop.py # odom and teleop

# Camera and false_base_link TF
ros2 run tf2_ros static_transform_publisher -0.1 0 0.336 0 0 0 robot_footprint D455_color_optical_frame
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0  robot_footprint
```