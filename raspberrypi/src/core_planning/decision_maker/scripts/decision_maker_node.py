#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from status import Status
from common.msg import Bbox, Lane

from decision_maker_core import DecisionMaker
from object_handler import ObjectHandler
from lane_handler import LaneHandler
from utils import set_twist


# 接收到车道线的数据后的处理
def lane_callback(bbox):
    l_handler.load_status(status, bbox)

# 接收到目标检测的数据后的处理
def object_callback(bbox):
    o_handler.load_status(status, bbox)


# 决策
def decision(twist, status):
    global all_decision
    global o_handler
    global l_handler

    # 没接收到消息时的操作
    threshold_duration = rospy.Duration(1, 0)  # s, ns 超时时长
    if status.have_data == False:
        if (rospy.Time.now() - status.time) > threshold_duration: # 超时，重置
            set_twist(twist, angle=0, speed=0)     # 设置转向角度为0，速度为0
        return twist

    # 接收到消息时的操作

    # 对车道线做决策
    twist, status = l_handler.process(twist, status)

    # 对物体做决策
    twist, status = o_handler.process(twist, status)

    # 总决策
    twist, status = all_decision.process(twist, status)

    return twist



if __name__ == "__main__":
    rospy.init_node('decision_maker_node')  # 初始化节点

    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)   # 发布控制消息
    object_sub = rospy.Subscriber('/perceive/object_detection', Bbox, object_callback)    # 订阅目标检测节点
    lane_sub = rospy.Subscriber('/perceive/lane_detection', Lane, lane_callback)    # 订阅车道线节点
    
    rate = rospy.Rate(10)   # 设置发布速率
    status = Status(rospy.Time.now())       # 创建状态
    twist = Twist()         # 控制命令消息

    all_decision = DecisionMaker()  # 总决策
    o_handler = ObjectHandler()    # 创建物体处理
    l_handler = LaneHandler()    # 创建车道线处理

    rospy.loginfo('[swucar] start decision...')
    while not rospy.is_shutdown():
        twist = decision(twist, status)
        vel_pub.publish(twist)      # 发布控制消息
        status.have_data = False    # 标志置False，为了判断后面的数据是否有接收
        rate.sleep()
