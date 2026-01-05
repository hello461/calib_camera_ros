### Control by file
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="file:///home/q/ros2_ws/read_yaml_parameter/test.yaml"
### Control by source code
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="package://./test.yaml"