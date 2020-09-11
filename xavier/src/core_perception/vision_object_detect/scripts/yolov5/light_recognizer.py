import numpy as np
import cv2

class LightRecognizer:
    '''
        传入图片，获取目标的边界框。
    '''
    # red/green light select
    img_bin_r = None
    img_bin_g = None
    map_classes = ['light red', 'light yellow', 'light green']

    # _color_filter
    # HSV (0-180, 0-255, 0-255)
    R_min1 = np.array([0, 77, 77])
    R_max1 = np.array([40, 255, 255])
    R_min2 = np.array([170, 77, 77])
    R_max2 = np.array([180, 255, 255])
    G_min = np.array([80, 77, 77])
    G_max = np.array([100, 255, 255])
    # _dilate_erode
    # erode:1 -> dilate:2  dilate:1 -> erode:3
    # near distance: iter_erode:3   far: iter_erode:1
    kernel_dilate = np.ones((3, 3), np.uint8)
    kernel_erode = np.ones((3, 3), np.uint8)
    iter_dilate = 1
    iter_erode = 1
    # _recognize_light
    color_rate = 0.2
    

    def __init__(self, use_erode_dilate=True):
        self.use_erode_dilate = use_erode_dilate

    def recognize(self, img):
        # img = img.copy()
        return self._recognize(img)

    def _recognize(self, img):
        '''
            img: (type cv2 Mat)

            return:
                bbox: [[x, y, w, h], ...]
        '''
        img_bin = self._color_filter(img)
        if self.use_erode_dilate:
            img_bin = self._dilate_erode(img_bin)
        color_cores = self._recognize_light(img_bin)

        # print(color_cores)

        return self.map_classes[color_cores.index(max(color_cores))]

    # def _color_white_balance(self, img):
    #     '''
    #         白平衡
    #     '''
    #     bgr = cv2.split(img)
    #     b = np.mean(bgr[0])
    #     g = np.mean(bgr[1])
    #     r = np.mean(bgr[2])

    #     kb = (b + g + r) / b / 3
    #     kg = (b + g + r) / g / 3
    #     kr = (b + g + r) / r / 3

    #     bgr[0] = bgr[0] * kb
    #     bgr[1] = bgr[1] * kg
    #     bgr[2] = bgr[2] * kr

    #     img = cv2.merge(bgr)
    #     img = img.clip(0, 255).astype(np.uint8)

    #     return img

    
    def _color_filter(self, imgBGR):
        '''
            颜色过滤
        '''
        imgHSV = cv2.cvtColor(imgBGR, cv2.COLOR_BGR2HSV)

        img_R_bin1 = cv2.inRange(imgHSV, self.R_min1, self.R_max1)
        img_R_bin2 = cv2.inRange(imgHSV, self.R_min2, self.R_max2)
        img_R_bin = np.maximum(img_R_bin1, img_R_bin2)

        img_G_bin = cv2.inRange(imgHSV, self.G_min, self.G_max)
        
        img_bin = np.maximum(img_G_bin, img_R_bin)

        return img_bin


    def _dilate_erode(self, img_bin):
        '''
            腐蚀膨胀
        '''
        img_bin = cv2.dilate(img_bin, self.kernel_dilate, iterations=self.iter_dilate)
        img_bin = cv2.erode(img_bin, self.kernel_erode, iterations=self.iter_erode)

        return img_bin


    def _recognize_light(self, img_bin):
        '''
            根据数据，判断红绿灯颜色
        '''
        color_cores = [0 for _ in range(3)]
        h, w = img_bin.shape

        if h / w > 2:   # 3个灯
            imgx = [img_bin[h//3*i:h//3*(i+1), :] for i in range(3)]
            color_cores = [len(img[img>0])/(w*h/3) for img in imgx]
            if color_cores[0] < self.color_rate and color_cores[2] < self.color_rate:   # 如果占比不大，可能是左右箭头
                color_cores = [0, 0.1, 0]
        else:   # 1/2个灯
            # TODO
            pass
        return color_cores 


if __name__ == "__main__":
    import os, sys

    in_path = sys.argv[1]
    if not os.path.exists(in_path):
        if in_path != '':
            print('文件不存在!')
            sys.exit(1)

    img = cv2.imread(in_path)

    lr = LightRecognizer()
    light = lr.recognize(img)

    print(light)