#!/usr/bin/env python3

import numpy as np
import rospy


# class Enum:
#     traffic_light = ['green', 'red', 'yellow']
#     traffic_sign = ['pedestrian_crossing', 'speed_limited', 'speed_unlimited']
#     names = ['person',
#             'bicycle',
#             'car',
#             'motorcycle',
#             'airplane',
#             'bus',
#             'train',
#             'truck',
#             'boat',
#             'light green',
#             'light red',
#             'light yellow',
#             'pedestrian crossing',
#             'parking area',
#             'traffic sign',
#             'cat',
#             'dog'
#     ]
#     target = [Target()]

class Target:
    classes = ''
    bbox = []
    confidence = 0
    width = 0   # 0~1.
    height = 0  # 0~1.
    distance = 0
    distance_lv = 0 # 0:危险 1：太近 2：适中 3：稍远（有影响） 4.太远（影响小） 5.没有影响
    havior = ''


class Status:
    # 标志
    have_data = False   # 是否接收到了感知数据

    # 时间
    time = 0            # rospy.Time.now()
    time_perceive = {}  # 感知的时间，记录每个检测到的时间

    # 感知
    lanes = np.zeros((4, 18, 2))   # 车道线坐标点
    lane_num = 0
    lane_on = 0

    traffic_light = '' # 信号灯状态

    traffic_sign = []   # 交通标志

    targets = []        # 目标

    # 决策
    lane_cur = -1

    # 执行
    steer_angle = 0 # -1. ~ 1.
    speed = 0   # 0 ~ 100

    def __init__(self, time=0):
        self.time = time
    
    def set_time(self):
        '''
            设置最后接收的感知消息的时间为当前状态的时间
        '''
        if self.time_perceive == {}:
            return False
        self.time = sorted(self.time_perceive.items(), key=lambda x: x[1])[0][1]
        return True

    def clear(self):
        self.have_data = False
        self.time_perceive.clear()

# status = Status()