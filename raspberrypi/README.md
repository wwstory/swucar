# 树莓派安装ROS

url: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi


在**Adding Released Packages**部分，需要添加`rosserial`。
```sh
rosinstall_generator rosserial --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall
```

> 注：编译时，不能设环境变量为python3。（`export ROS_PYTHON_VERSION=3`）