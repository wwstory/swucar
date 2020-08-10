import numpy as np
import torch
from random import randint

from utils.general import *

class Detect:

    weights = ''
    img_size = 640
    conf_thres = 0.4
    iou_thres = 0.5
    device = ''
    classes = None
    agnostic_nms = False
    augment = False
    names = None
    is_classify = False
    img_size = 640

    def __init__(self, weights='weights/yolov5s.pth'):
        self.weights = weights

        # Initialize
        device = select_device(self.device)
        self.device = device
        self.half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        # 1
        # yaml_conf = 'models/yolov5s.yaml'
        # model = Model(yaml_conf).to(device)
        # model.load_state_dict(torch.load(weights, map_location=device))
        # model.to(device).eval()
        # 2
        model = torch.load(weights, map_location=device)['model'].float()  # load to FP32
        # 3
        # model = attempt_load(weights, map_location=device)  # load FP32 model
        imgsz = check_img_size(self.img_size, s=model.stride.max())  # check img_size
        if self.half:
            model.half()  # to FP16
        
        # Second-stage classifier
        if self.is_classify:
            pass
            # modelc = load_classifier(name='resnet101', n=2)  # initialize
            # modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model'])  # load weights
            # modelc.to(device).eval()

        # Get names and colors
        self.names = model.module.names if hasattr(model, 'module') else model.names
        self.colors = [[randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]

        # Run inference
        img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
        _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once

        self.model = model


    def detect(self, img):
        # Origin
        img0_shape = img.shape

        # Convert
        img = self._convert_img(img)
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img, augment=self.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

        # Convert pred to box
        bbox = self._convert_pred(pred, img, img0_shape)

        # Apply Classifier
        if self.is_classify:
            # pred = apply_classifier(pred, modelc, img, im0s)
            pass

        return bbox

    def __call__(self, img):
        return self.detect(img)

    def _convert_img(self, img):
        # Padded resize
        img = letterbox(img, new_shape=416)[0]
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        # Tensor
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        return img

    def _convert_pred(self, pred, img, img0_shape):
        bbox = []

        if pred[0] is None:
            return bbox

        # Rescale boxes from img_size to im0 size
        for i, det in enumerate(pred):  # detections per image
            if det is not None and len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0_shape).round()

        # Repack
        h0, w0, _ = img0_shape
        for p in pred[0].data.tolist():
            x1, y1, x2, y2 = p[:4]
            conf = p[4]
            classes_index = int(p[5])
            classes_name = self.names[classes_index]
            bbox.append([x1/w0, y1/h0, x2/w0, y2/h0, classes_name, conf])   # 绝对坐标 -> 0~1
        return bbox

    def show(self, img, bbox, out_path='out.jpg'):
        h, w, _ = img.shape
        for *xyxy, classes, conf in bbox:
            x1, y1, x2, y2 = xyxy
            xyxy = [x1*w, y1*h, x2*w, y2*h] # 0~1 -> 绝对坐标
            label = '%s %.2f' % (classes, conf)
            plot_one_box(xyxy, img, label=label, color=self.colors[self.names.index(classes)], line_thickness=2)
        cv2.imwrite(out_path, img)
    

if __name__ == '__main__':
    import cv2

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

    d = Detect('weights/yolov5s17.pt')
    
    img = cv2.imread('/tmp/car/11.jpg')
    bbox = d(img)
    d.show(img, bbox)

    print(bbox)