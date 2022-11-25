import cv2 as cv
import numpy as np
import os 
import glob
import math
from typing import Tuple, List, Dict
import matplotlib.pyplot as plt
import errno

class Camera():
    # This is the class of camera that attatched to TM5-900 robot arm

    # initialize the camera
    def __init__(self, config: Dict):
        self.in_matrix: np.ndarray = None       # Intrinsic matric
        self.eulerAng: np.ndarray = None        # Eular angle position
        self.rgbimg: np.ndarray = None          # RGB image
        self.gray: np.ndarray = None            # Gray image
        self.binarized_img: np.ndarray = None   # Binerized image
        self.label_map: np.ndarray = None       # Label map
        self.objects: List[Dict] = None         # List of objects
        self.contours: List[np.ndarray] = None  # List of contours
        self.config = config                    # Configuration
    
    # load the image that received from the robot arm server
    def load_img(self, cv_img:np.array):
        self.rgbimg = cv_img
        self.gray = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY).astype(np.uint8)
        self.binarized_img = np.zeros_like(self.gray).astype(np.uint8)

    # turn the gray image to binerized image with respect to the threshold
    def binarization(self):
        [h, w] = self.gray.shape
        for y in range(h):
            for x in range(w):
                if self.gray[y][x] >= self.config['bi_thres']:
                    self.binarized_img[y][x] = 255

    # find the object(s) in the binerized image
    def find_objects(self):
        contours, _ = cv.findContours(image=self.binarized_img, mode=cv.RETR_LIST, method=cv.CHAIN_APPROX_NONE)
        self.contours = sorted(contours, key = lambda cnt : -cv.contourArea(cnt))[0:self.config['number_thres']]
        self.objects = []
        self.label_map = np.zeros_like(self.binarized_img)

        for cnt in self.contours[:5]:
            if cv.contourArea(cnt) >= self.config['object_thres']:
                cv.drawContours(self.label_map, [cnt], -1, color = (255, 255, 255), thickness = cv.FILLED)
                obj = dict()
                M = cv.moments(cnt)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                phi = 0.5 * math.atan2(2 * M['nu11'], M['nu20'] - M['nu02'])
                # print(f"{cx, cy}, angle: {phi * 180 / math.pi}")
                obj['center'] = (cx, cy)
                obj['orientation'] = phi * 180 / math.pi
                self.objects.append(obj)

    def draw_contours(self, axis: plt.Axes):
        cv.drawContours(self.label_map, self.contours, -1, color = (255, 255, 255), thickness = cv.FILLED)
        for obj in self.objects:
            slope = math.tan(obj['orientation'] * math.pi / 180)
            (cx, cy) = obj['center']
            L_point = (0, -cx * slope + cy)
            R_point = (self.label_map.shape[1], (self.label_map.shape[1] - cx) * slope + cy)
            axis.plot((L_point[0], R_point[0]), (L_point[1], R_point[1]), color = 'b', linewidth = 1)

    # update camera position and object detection's result
    def update(self, img: np.ndarray, eularAng: np.ndarray):
        self.load_img(img)
        self.binarization()
        self.eularAng = eularAng
        self.find_objects()

    def calibration(self):
        # TODO use tempalte image board to calculate intrinsic matrix
        if os.path.exists(self.config['mtx_file']):
            self.in_matrix = np.load(self.config['mtx_file'])
        else:
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), self.config['mtx_file'])

if __name__ == '__main__':
    camera = Camera()
