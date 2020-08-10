
# lib

- vision_opencv: https://github.com/ros-perception/vision_opencv

# 打开uvc摄像头

url: http://wiki.ros.org/uvc_camera

```sh
sudo apt-get install ros-melodic-uvc-camera

roscore
rosrun uvc_camera uvc_camera_node
#rosrun uvc_camera uvc_camera_node _device:=/dev/video2
```