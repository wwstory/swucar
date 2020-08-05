
# 控制

```sh
roscore

rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

rostopic echo /cmd_vel

rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0, 0, 0]' '[0, 0, 0]'
```

# doc

[arduino pro mini 官方文档](https://store.arduino.cc/usa/arduino-pro-mini)

