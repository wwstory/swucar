# introduce
```sh
roslaunch realsense2_camera rs_camera.launch
```
- `realsense2_camera`发布rgb和depth图像`/camera/color/image_raw`和`/camera/depth/image_rect_raw`等。
- `main.py`处理`/camera/color/image_raw`话题的图片，并发布定位数据到`/image_detection`话题。

# install
## pc (训练)

安装ros：
http://wiki.ros.org/melodic/Installation/Ubuntu

安装realsense sdk：
https://www.intelrealsense.com/developers/

安装realsense-ros：
https://github.com/IntelRealSense/realsense-ros

安装pytorch


## jetson xavier
- https://github.com/jetsonhacks/installRealSenseSDK
- https://github.com/jetsonhacks/installROS
- https://github.com/jetsonhacks/installRealSenseROS
- https://github.com/jetsonhacks/buildOpenCVXavier

安装pytorch：
https://elinux.org/Jetson_Zoo

> 编译opencv问题：`catastrophic error: error while writing generated C++ file: No space left on device`，空间不够

## python库
```
torch
torchvision
PyYAML
rospy
rospkg
pillow<7.0.0
opencv
```

> opencv：使用nvidia官方给的编译脚本安装 (无须自己下载opencv.git)


# run
如果要运行，需要在`image_detect/scripts/checkpoints/`目录放入SSD训练好的模型。（此处的模型名称`config.py`中设为了`COCO.pth`）
## use
```sh
. ~/swucar/devel/setup.bash
roscore
roslaunch realsense2_camera rs_camera.launch
rosrun image_detect main.py
rostopic echo /image_detection
```

## test
```sh
. ~/swucar/devel/setup.bash
roscore
roslaunch realsense2_camera rs_camera.launch
rosrun image_detect test_plot.py
rqt
```

> rqt选择，Plugins -> Visualization -> Image Viewer， 选择/image_detection节点。

# ref
ros发布Image类型数据：
https://blog.csdn.net/lucky__ing/article/details/79949294

ros监听多个topic：
https://blog.csdn.net/zlb_zlb/article/details/103444360

xavier安装pytorch：
https://elinux.org/Jetson_Zoo
https://devtalk.nvidia.com/default/topic/1041716/jetson-agx-xavier/pytorch-install-problem/
https://devtalk.nvidia.com/default/topic/1049071/jetson-nano/pytorch-for-jetson-nano/
https://github.com/pytorch/pytorch/issues/8103


