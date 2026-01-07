### Note:
Robot must be stand down to image can smooth connect to robot's image

### Install required packages
sudo apt install ros-humble-camera-ros
sudo apt install ros-humble-image-view
sudo apt install ros-humble-camera-calibration
sudo apt install ros-humble-image-proc

### Control by file
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="file:///home/q/ros2_ws/read_yaml_parameter/test.yaml"
### Control by source code
ros2 run camera_info camera_info_publisher --ros-args -p camera_info_url:="package://./test.yaml"

### Calibrate camera
ros2 run camera_calibration cameracalibrator --no-service-check --size 8x6 --square 0.025 image:=/camera/image_raw
ros2 run camera_calibration cameracalibrator --no-service-check --size 6x4 --square 0.037 image:=/camera/image_raw

