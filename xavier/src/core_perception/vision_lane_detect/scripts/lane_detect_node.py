#!/usr/bin/env python3

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_lane_detect.msg import Lane


class LaneDetectNode:
    
    def __init__(self, weights_path):
        self.lane_detector = LaneDetector(weights_path)
        self.bridge = CvBridge()

        self.img_sub = rospy.Subscriber('/image_raw_rgb', Image, self.callback, queue_size=3)
        self.lane_pub = rospy.Publisher('/perceive/lane_detection', Lane, queue_size=3)

    def start(self):
        rospy.loginfo('[swucar] start lane detect ...')
        rospy.spin()

    def callback(self, data):
        # 转换图片
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridge as e:
            print(e)

        if img is None:
            return

        # 装载数据
        lanes = self.lane_detector(img)
        lane_msg = Lane()
        lane_msg.header.stamp = rospy.Time.now()
        lane_msg.num = len(lanes)
        lane_msg.data = lanes.flatten().tolist()

        # 发送
        self.lane_pub.publish(lane_msg)


if __name__ == "__main__":
    rospy.init_node('lane_detect_node')
    ufld_path = rospy.get_param('~ufld_path')

    # load ufld sys path
    import sys
    sys.path.append(ufld_path)
    from lane_detector import LaneDetector

    weights_path = rospy.get_param('~weights_path')
    
    LaneDetectNode(weights_path).start()