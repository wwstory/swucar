import cv2
import torch
from torch.nn.functional import softmax
import numpy as np
import torchvision.transforms as T

from model.model import parsingNet


class LaneDetector:

    def __init__(self, weights_path='./weights/culane_18.pth'):
        # 设置参数
        torch.backends.cudnn.benchmark = True
        cls_num_per_lane = 18
        self.griding_num = 200
        backbone = '18'

        # 加载模型
        net = parsingNet(pretrained = False, backbone=backbone,cls_dim = (self.griding_num+1,cls_num_per_lane,4), use_aux=False)
        state_dict = torch.load(weights_path, map_location='cpu')['model']
        compatible_state_dict = {}
        for k, v in state_dict.items():
            if 'module.' in k:
                compatible_state_dict[k[7:]] = v
            else:
                compatible_state_dict[k] = v
        net.load_state_dict(compatible_state_dict, strict=False)
        net.eval()
        if torch.cuda.is_available():
            net.cuda()
        self.net = net

        # 图像变换
        self.img_transforms = T.Compose([
            T.ToTensor(),
            T.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])


    def __call__(self, cv_img):
        # 图片预处理
        img = self._preprocess(cv_img)

        # 预测
        with torch.no_grad():
            out = self.net(img)

        # 后期处理
        lanes = self._postprocess(out)

        # 显示
        # self.show(cv_img, lanes)

        return lanes


    def _preprocess(self, img):
        self.rate = 0.0339

        img = cv2.resize(img, (800, 288))
        img = np.ascontiguousarray(img[:, :, ::-1])               # BGR to RGB
        img = self.img_transforms(img)
        img = img.unsqueeze(0)
        if torch.cuda.is_available():
            img = img.cuda()

        return img


    def _postprocess(self, out):
        # 选出最优的预测点
        _out = out[0].data.cpu().numpy()
        _out = _out[:, ::-1, :]
        prob = softmax(torch.from_numpy(np.ascontiguousarray(_out[:-1, :, :])), dim=0).numpy()
        idx = np.arange(self.griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        _out = np.argmax(_out, axis=0)    # 201选1
        loc[_out == self.griding_num] = 0

        lanes = []
        for col in range(loc.shape[1]):
            lane = []
            # if np.sum(loc[:, col] != 0) > 2:  # 这条道有点
            for row in range(loc.shape[0]):
                if loc[row, col] > 0:   # 这个格子有点
                    # point = (int(loc[row, col] / self.griding_num * self.img_width ) - 1, int(self.img_height - row * (self.rate * self.img_height)) - 1)
                    point = (loc[row, col] / self.griding_num, 1 - row * self.rate)
                else:
                    point = (0., 0.)
                lane.append(point)
            lanes.append(lane)
        return np.array(lanes)  # shape: col, row, point(x,y)


    def show(self, img, lanes, out_path='/tmp/out.jpg'):
        h, w, _ = img.shape
        for lane in lanes:
            for point in lane:
                x, y = point
                x *= w
                y *= h
                cv2.circle(img, (x, y), 5, (70, 70, 255), -1)
        # cv2.imshow('', img)
        # cv2.waitKey(3000)
        cv2.imwrite(out_path, img)


if __name__ == "__main__":
    ld = LaneDetector('./weights/culane_18.pth')
    
    img = cv2.imread('/tmp/test.jpg')
    ld(img)
