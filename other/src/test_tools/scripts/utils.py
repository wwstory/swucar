import cv2
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


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
    return img


def read_lane_msg(msg):
    '''
        lane_msg -> lane
    '''
    num = msg.num
    lanes = np.array(msg.data)
    lanes = lanes.reshape(4, -1, 2)
    return lanes


def plot_lane_dot(img, lanes):
    for lane in lanes:
        for point in lane:
            x, y = point
            cv2.circle(img, (x, y), 3, (70, 70, 255), -1)
    return img