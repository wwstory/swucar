from yolov5.utils.datasets import *
from yolov5.utils.utils import *
from yolov5.models.yolo import Model

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

    def __init__(self, weights='weights/yolov5s.pth', yaml_conf='models/yolov5s.yaml', names=None):
        self.weights = weights

        # Initialize
        device = torch_utils.select_device(self.device)
        self.device = device
        self.half = device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        model = Model(yaml_conf).to(device)
        model.load_state_dict(torch.load(weights))
        # model = torch.load(weights, map_location=device)['model'].float()  # load to FP32
        # torch.save(torch.load(weights, map_location=device), weights)  # update model if SourceChangeWarning
        # model.fuse()
        model.to(device).eval()
        if self.half:
            model.half()  # to FP16

        # Get colors
        assert names is not None, 'names must'
        self.names = names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.names))]
        
        self.model = model


    def detect(self, img):
        # Origin
        # im0 = img.copy()    # show
        img0_shape = img.shape

        # Convert
        img = self._convert_img(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img, augment=self.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

        # # Show
        # # Process detections
        # for i, det in enumerate(pred):  # detections per image
        #     if det is not None and len(det):
        #         # Rescale boxes from img_size to im0 size
        #         det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

        #         for *xyxy, conf, cls in det:
        #             label = '%s %.2f' % (self.names[int(cls)], conf)
        #             plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=3)

        # cv2.imwrite('/tmp/out.jpg', im0)
        # cv2.imshow('', im0)
        # if cv2.waitKey(3000) == ord('q'):  # q to quit
        #     raise StopIteration

        bbox = self._convert_pred(pred, img, img0_shape)
        return bbox

    def __call__(self, img):
        return self.detect(img)

    def _convert_img(self, img):
        # Padded resize
        img = letterbox(img, new_shape=416)[0]
        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
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
        for p in pred[0].data.tolist():
            x1, y1, x2, y2 = p[:4]
            conf = p[4]
            classes_index = int(p[5])
            classes_name = self.names[classes_index]
            bbox.append([x1, y1, x2, y2, classes_name, conf])
        return bbox


if __name__ == '__main__':
    import cv2

    names = [
        'red_stop',
        'green_go',
        'yellow_back',
        'pedestrian_crossing',
        'speed_limited',
        'speed_unlimited',
    ]

    d = Detect('yolov5/models/yolov5s.yaml', 'yolov5/weights/yolov5s.pth', names)
    
    img = cv2.imread('/tmp/test.jpg')
    bbox = d(img)

    print(bbox)