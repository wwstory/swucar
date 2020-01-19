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
# test
import matplotlib.pyplot as plt
import time
from data import COCO_CLASSES as CLASSES



class ImageDetectMark:

    count = 0

    def __init__(self):
        rospy.init_node('image_detect_mark_node')
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback)

        # ssd
        self.detector = Detect()
        rospy.loginfo('start...')

    def callback(self, image):
        img = np.frombuffer(image.data, dtype=np.uint8).reshape((480, 640, 3))
        detection = self.detector(img)
        pos, classes, scores = self.separate_data(detection)

        # plot
        plt.imshow(img)
        scale = np.array([img.shape[1], img.shape[0], img.shape[1], img.shape[0]])
        for p, c, s in zip(pos, classes, scores):
            class_name = CLASSES[int(c)]
            score = s
            x1, y1, x2, y2 = np.array(p) * scale
                
            x, y, w, h = x1, y1, x2-x1, y2-y1
            plt.gca().add_patch(plt.Rectangle((x, y), w, h, fill=False))
            txt = f'{class_name}:{round(score*100,2)}%'
            plt.text(x, y, txt)
        plt.axis('off')
        plt.savefig(f'/tmp/{time.strftime(f"%Y-%m-%d_%H:%M:%S_{self.count}.png", time.localtime())}')
        plt.clf()
        self.count += 1

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
        # return list(pos.reshape(-1)), list(classes.astype(int)), list(scores)
        return pos, classes, scores


if __name__ == "__main__":
    ImageDetectMark().start()
    