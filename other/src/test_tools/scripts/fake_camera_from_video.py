#!/usr/bin/env python3


'''
    can use cv_bridge by build cv_bridge in python 3.

    -----------
    deprecated
    ps:
    only available by python 2,
    when use python3, cv_bridge still use python 2,
    this cause cv2_to_imgmsg() use 'bgr8' is error.
'''


import rospy
from sensor_msgs.msg import CompressedImage
from ww_cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import os



def clean_up():
    rospy.loginfo('[swucar] stop pub test video!')


if __name__ == "__main__":
    rospy.init_node('fake_camera_node')
    rospy.on_shutdown(clean_up)

    pub = rospy.Publisher('/image_raw/compressed', CompressedImage, queue_size=1)

    bridge = CvBridge()

    video_path = rospy.get_param('test_file_path')
    play_rate = rospy.get_param('~rate', default=24)

    rate = rospy.Rate(play_rate)

    rospy.loginfo('[swucar] start pub fake video...')
    ret = False
    while not rospy.is_shutdown():
        if os.path.exists(video_path) and not ret:
            cap = cv2.VideoCapture(video_path)
        elif not ret:
            rate.sleep()
            continue
        ret, img = cap.read()
        if not ret:
            rospy.loginfo('play finish!')
            continue

        try:
            img_msg = bridge.cv2_to_compressed_imgmsg(img)
            img_msg.header.stamp = rospy.Time.now()
            pub.publish(img_msg)
        except CvBridgeError as e:
            print(e)
        
        rate.sleep()
