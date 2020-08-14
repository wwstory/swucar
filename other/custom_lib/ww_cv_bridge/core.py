import numpy as np

from sensor_msgs.msg import Image


class CvBridgeError(TypeError):
    pass

class CvBridge:
    def imgmsg_to_cv2(self, img_msg, desired_encoding='bgr8'):
        assert isinstance(img_msg, Image), 'input type must is sensor_msgs.Image'

        w, h = img_msg.width, img_msg.height
        if desired_encoding == 'bgr8':
            # img = np.frombuffer(img_msg.data, np.uint8).reshape(h, w, 3)
            img = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=np.uint8, buffer=img_msg.data)
        else:
            img = None
        return img


    def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
        assert isinstance(cv_image, np.ndarray), 'input type must is numpy.ndarray'

        h, w, _ = cv_image.shape
        img_msg = Image()
        img_msg.data = cv_image.tostring()  # cv_image.tobytes()
        img_msg.width = w
        img_msg.height = h
        if encoding == 'bgr8':
            img_msg.encoding = encoding
        if cv_image.dtype.byteorder == '>':
            img_msg.is_bigendian = True
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg