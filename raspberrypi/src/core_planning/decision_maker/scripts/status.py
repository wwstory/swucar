#!/usr/bin/env python3

import numpy as np

class Status:
    enum = Enum()

    lane_cur = -1
    lane_num = 0
    lane_on = 0

    traffic_light = 'green' # 信号灯状态

    traffic_sign = []   # 交通标志

    objects = []        # 目标

class Enum:
    traffic_light = ['red', 'green', 'yellow']
    traffic_sign = ['pedestrian_crossing', 'speed_limited', 'speed_unlimited']
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




status = Status()