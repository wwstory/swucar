#!/usr/bin/env python3
#coding=utf-8

# ros
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from image_detect.msg import Detection

# need
import numpy as np
# test
import matplotlib.pyplot as plt
import time
from data import COCO_CLASSES as CLASSES
from PIL import Image as PImage


class ImageMark:

    image = None
    detection = None

    IMAGE_WIDHT = 640
    IMAGE_HEIGHT = 480

    def __init__(self):
        rospy.init_node('image_mark_node')
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_img)
        rospy.Subscriber('/image_detection', Detection, self.callback_detection)
        self.pub = rospy.Publisher('image_mark', Image , queue_size=10)
        rospy.loginfo('start...')

    def callback_img(self, image):
        img = np.frombuffer(image.data, dtype=np.uint8).reshape((self.IMAGE_HEIGHT, self.IMAGE_WIDHT, 3))
        self.image = img
        
    def callback_detection(self, detection):
        if self.image is None:
            return
        
        self.detection = detection
        img = self.mark(self.image, self.detection)
        img_mark = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'img'
        img_mark.width = self.IMAGE_WIDHT
        img_mark.height = self.IMAGE_HEIGHT
        img_mark.encoding = 'rgb8'
        img_mark.data = np.array(img).tostring()
        img_mark.header = header
        img_mark.step= self.IMAGE_WIDHT * 3 # image_width * image_channel
        self.pub.publish(img_mark)

        self.image = None

    def start(self):
        rospy.spin()
    
    def separate_data_from_detection(self, detection):
        '''
        detection:
            pos: list [x1, y1, x2, y2, _x1...]
            classes: list [classes1, classes2, ...]
            scores: list [score1, score2]
        
        return:
            pos: numpy [[x1, y1, x2, y2], [_x1...]]
            classes: numpy [classes1, classes2, ...]
            scores: numpy [score1, score2]
        '''
        if detection == []: # no object
            return [], [], []
        
        data = np.array(detection)
        pos, classes, scores = np.array(detection.pos), np.array(detection.classes), np.array(detection.scores)
        # return list(pos.reshape(-1)), list(classes.astype(int)), list(scores)
        return pos.reshape(-1, 4), classes, scores
    
    def mark(self, img, detection):
        img = np.array(img)
        # img.flags.writeable = True
        pos, classes, scores = self.separate_data_from_detection(detection)

        txt = ''
        scale = np.array([img.shape[1], img.shape[0], img.shape[1], img.shape[0]])
        for p, c, s in zip(pos, classes, scores):
            class_name = CLASSES[int(c)]
            score = s
            x1, y1, x2, y2 = np.array(p) * scale
                
            img = self.rectangle(img, x1, y1, x2, y2, color=(255, 0, 0))
            txt += f'{class_name}:{round(score*100,2)}%\t'
        rospy.loginfo(txt)
        
        return img
    
    def rectangle(self, img, x1, y1, x2, y2, color=(0, 0, 0), thickness=1, fill=False):
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        t = int(thickness)

        if fill:
            img[y1:y2, x1:x2, :] = color
        else:
            img[y1:y1+t, x1:x2, :] = color
            img[y2-t:y2, x1:x2, :] = color
            img[y1:y2, x1:x1+t, :] = color
            img[y1:y2, x2-t:x2, :] = color
        return img
    

if __name__ == "__main__":
    ImageMark().start()
    