#!/usr/bin/env python3

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_object_detect.msg import Bbox
from ww_cv_bridge import CvBridge, CvBridgeError


class ObjectDetectNode:
    
    def __init__(self, weights_path, is_classify = False):
        self.object_detector = Detect(weights_path, is_classify)
        self.bridge = CvBridge()

        self.img_sub = rospy.Subscriber('/image_raw_rgb', Image, self.callback, queue_size=3)
        self.object_pub = rospy.Publisher('/perceive/object_detection', Bbox, queue_size=3)

    def start(self):
        rospy.loginfo('[swucar] start object detect ...')
        rospy.spin()

    def callback(self, data):
        # 转换图片
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridge as e:
            print(e)

        if img is None:
            return

        # 装载数据
        bbox = self.object_detector(img)
        bbox_msg = self.write_msg(bbox)

        # 发送
        self.object_pub.publish(bbox_msg)

    def write_msg(self, bbox):
        '''
            bbox -> bbox_msg
        '''
        msg = Bbox()
        msg.header.stamp = rospy.Time.now()

        classes = []
        boxes = []
        confidence = []
        for b in bbox:
            classes.append(String(b[4]))    # ros String()
            boxes.extend(b[:4])
            confidence.append(b[-1])
        msg.num = len(classes)
        msg.classes = classes
        msg.boxes = boxes
        msg.confidence = confidence
        return msg
    
    def read_msg(self, msg):
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


if __name__ == "__main__":
    rospy.init_node('object_detect_node')
    yolo_path = rospy.get_param('~yolov5_path')

    # load yolov5 sys path
    import sys
    sys.path.append(yolo_path)
    from my_detect import Detect

    weights_path = rospy.get_param('~weights_path')
    is_classify = rospy.get_param('~is_classify', default=False)
    
    ObjectDetectNode(weights_path, is_classify).start()