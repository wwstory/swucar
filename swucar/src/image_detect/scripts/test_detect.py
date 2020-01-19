#!/usr/bin/env python3
#coding=utf-8


from detect import Detect
import numpy as np


def separate_data(detection):
    if detection == []: # no object
        return [], [], []
    data = np.array(detection)
    pos, classes, scores = data[:, :4], data[:, 4], data[:, -1]
    return list(pos.reshape(-1)), list(classes.astype(int)), list(scores)


if __name__ == "__main__":

    detector = Detect()
    img = np.random.rand(300, 300, 3)
    pos, classes, scores = separate_data(detector(img))
    print(pos, classes, scores)
