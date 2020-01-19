# introduce
```sh
roslaunch realsense2_camera rs_camera.launch
```
- `realsense2_camera`发布rgb和depth图像`/camera/color/image_raw`和`/camera/depth/image_rect_raw`等。
- `main.py`处理`/camera/color/image_raw`话题的图片，并发布定位数据到`/image_detection`话题。

# install
## pc

安装ros：
http://wiki.ros.org/melodic/Installation/Ubuntu

安装realsense sdk：
https://www.intelrealsense.com/developers/

安装realsense-ros：
https://github.com/IntelRealSense/realsense-ros

安装pytorch

## nvidia xavier

安装ros：
http://wiki.ros.org/melodic/Installation/Ubuntu

安装realsense sdk：
https://github.com/jetsonhacks/installRealSenseSDK

安装realsense-ros：
https://github.com/IntelRealSense/realsense-ros

安装pytorch：
https://www.ncnynl.com/archives/201903/2901.html

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


