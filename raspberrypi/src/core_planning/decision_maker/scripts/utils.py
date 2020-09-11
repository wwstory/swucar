import numpy as np
import random

def read_bbox_msg(msg):
    '''
        bbox_msg -> bbox
    '''
    bbox = []

    classes =[c.data for c in msg.classes]
    boxes = msg.boxes
    confidence = msg.confidence
    for i in range(len(classes)):
        bbox.append([boxes[i*4], boxes[i*4+1], boxes[i*4+2], boxes[i*4+3], classes[i], confidence[i]])
    return bbox


def read_lane_msg(msg):
    '''
        lane_msg -> lane
    '''
    num = msg.num
    lanes = np.array(msg.data)
    lanes = lanes.reshape(4, -1, 2)
    return lanes


def set_twist(twist, angle, speed):
    twist.linear.x = speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = angle
    return twist