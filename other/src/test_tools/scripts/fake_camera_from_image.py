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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import os



def clean_up():
    rospy.loginfo('[swucar] stop pub test image!')


if __name__ == "__main__":
    rospy.init_node('/fake/fake_camera_node')
    rospy.on_shutdown(clean_up)

    pub = rospy.Publisher('image_raw', Image, queue_size=1)
    rate = rospy.Rate(30)

    bridge = CvBridge()

    rospy.loginfo('[swucar] start pub fake image...')
    while not rospy.is_shutdown():
        img_path = rospy.get_param('test_file_path')
        if not os.path.exists(img_path):
            rate.sleep()
            continue
        img = cv2.imread(img_path)
        try:
            img_ros = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_ros.header.stamp = rospy.Time.now()
            pub.publish(img_ros)
        except CvBridgeError as e:
            print(e)
        
        rate.sleep()
