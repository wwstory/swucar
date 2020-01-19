#!/usr/bin/env python3
#coding=utf-8

# ros
import rospy
from sensor_msgs.msg import Image
from image_detect.msg import Detection

# need
import numpy as np
import torch
# ssd
from detect import Detect



class ImageDetect:

    def __init__(self):
        # ssd
        self.detector = Detect()

        rospy.init_node('image_detect_node')
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.pub = rospy.Publisher('image_detection', Detection , queue_size=10)
        rospy.loginfo('start...')

    def callback(self, image):
        img = np.frombuffer(image.data, dtype=np.uint8).reshape((480, 640, 3))
        detection = self.detector(img)
        pos, classes, scores = self.separate_data(detection)

        # rospy.loginfo(detection)
        self.pub.publish(Detection(pos, classes, scores))

    def start(self):
        rospy.spin()
    
    def separate_data(self, detection):
        '''
        detection: list [[x1, y1, x2, y2, classes, score], ...]
        '''
        if detection == []: # no object
            return [], [], []
        data = np.array(detection)
        pos, classes, scores = data[:, :4], data[:, 4], data[:, -1]
        return list(pos.reshape(-1)), list(classes.astype(int)), list(scores)


if __name__ == "__main__":
    ImageDetect().start()
    
