#!/usr/bin/env python3

import rospy
from ww_cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from test_tools.msg import Bbox, Lane
import message_filters

import cv2
from utils import *
import random

bridge = CvBridge()
speed, angle = 0., 0.

def twist_callback(twist):
    global speed
    global angle
    speed, angle = get_speed_and_angle(twist)

names = ['person',
        'bicycle',
        'car',
        'motorcycle',
        'airplane',
        'bus',
        'train',
        'truck',
        'boat',
        'light green',
        'light red',
        'light yellow',
        'pedestrian crossing',
        'parking area',
        'traffic sign',
        'cat',
        'dog'
]
filter_classes_list = ['train']
colors = {}
for name in names:
    colors[name] = [random.randint(0, 255) for _ in range(3)]

def all_callback(img, bbox_msg, lane_msg):
    global speed
    global angle

    try:
        img = bridge.imgmsg_to_compressed_cv2(img)
    except CvBridgeError as e:
        print(e)
    
    h, w, _ = img.shape

    # mark object
    if bbox_msg.num > 0:    # 有物体
        bbox = read_bbox_msg(bbox_msg)
        for *xyxy, classes, conf in bbox:
            if classes not in filter_classes_list:  # 过滤掉不显示的目标
                x1, y1, x2, y2 = xyxy
                xyxy = [x1*w, y1*h, x2*w, y2*h] # 0~1 -> 绝对坐标
                label = '%s %.2f' % (classes, conf)
                plot_one_box(xyxy, img, label=label, color=colors[classes], line_thickness=2)
    
    # mark lane
    if lane_msg:
        lane_num = lane_msg.num
        lanes = lane_msg.data
        lanes = np.array(lanes).reshape(lane_num, -1, 2)
        lanes[:,:,0] *= w   # 相对坐标 -> 绝对坐标
        lanes[:,:,1] *= h
        lanes = lanes.astype(int)
        plot_lane_dot(img, lanes)
    
    # mark process bar
    rospy.loginfo(f'{speed} {angle}')
    plot_process_bar(img, angle, length=0.7, location=(0.5, 0.9), thickness=0.02, grap=0.005, front_color=get_color_for_val(angle), background_color=(0, 0, 0))
    plot_process_bar(img, speed, length=0.7, location=(0.5, 0.95), thickness=0.02, grap=0.005, front_color=get_color_for_val(speed), background_color=(0, 0, 0))

    img_mark = bridge.compressed_cv2_to_imgmsg(img)
    img_mark.header.stamp = rospy.Time.now()

    img_mark_pub.publish(img_mark)


if __name__ == "__main__":
    rospy.init_node('visual_mark_object_lane_twist_node')

    img_sub = message_filters.Subscriber('image_raw/compressed', CompressedImage, queue_size=1)
    bbox_sub = message_filters.Subscriber('/perceive/object_detection', Bbox, queue_size=1)
    lane_sub = message_filters.Subscriber('/perceive/lane_detection', Lane, queue_size=1)

    twist_sub = rospy.Subscriber('/cmd_vel', Twist, twist_callback, queue_size=1)


    img_mark_pub = rospy.Publisher('/visual/image_mark_raw/compressed', CompressedImage, queue_size=1)

    rospy.loginfo('[swucar] start mark objectl/lane/twist on image...')

    # ts = message_filters.TimeSynchronizer([img_sub, bbox_sub, lane_sub], 10)
    ts = message_filters.ApproximateTimeSynchronizer([img_sub, bbox_sub, lane_sub], queue_size=10, slop=0.1, allow_headerless=False)
    ts.registerCallback(all_callback)

    rospy.spin()
    
