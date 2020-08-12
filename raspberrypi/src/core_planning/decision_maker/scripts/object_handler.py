#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from common.msg import Bbox
from status import Target

from utils import read_bbox_msg, set_twist

class ObjectHandler:

    def load_status(self, status, bbox_msg):
        status.have_data = True                 # 接收到了数据，置为True
        if bbox_msg.num > 0:    # 有物体
            bbox = read_bbox_msg(bbox_msg)      # 将msg读取出来
            for *xyxy, classes, conf in bbox:   # 装填目标检测的结果到status中
                x1, y1, x2, y2 = xyxy

                target = Target()
                target.classes = classes
                target.bbox = xyxy
                target.confidence = conf
                target.width = x2 - x1
                target.height = y2 - y1

                status.targets.append(target)
        status.time_perceive['object'] = rospy.Time.now()   # 记录目标检测到的时间

    # TODO
    def process(self, twist, status):
        return twist, status