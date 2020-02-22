import cv2
import yaml
import pyzed.sl as sl
import math


class Detector:
    def __init__(self, yml_data, yml_path, nt):
        self.yml_path = yml_path
        self.yml_data = yml_data
        self.frame = 0
        self.nt = nt

    def preProcessFrame(self, frame):
        lower = self.yml_data['color']['lower']
        upper = self.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        image = frame[0:int(h / 2), 0:w]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def reloadData(self):
        with open(self.yml_path, 'r') as file:
            self.yml_data = yaml.safe_load(file)
    def findTarget(self, mask, zed, point_cloud):
        # Returns contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) != 0:
            c = max(contours, key=cv2.contourArea)
            rect = cv2.boundingRect(c)
        else:
            self.nt.clearTable()
            return -1
        if c.any():
            cx = rect[0] + (rect[2] * .5)
            cy = rect[1] + (rect[3] * .5)
            self.nt.putValue('center_x', cx)
            self.nt.putValue('center_y', cy)

            err, point3D = point_cloud.get_value(cx, cy)
            distance = math.sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1] + point3D[2] * point3D[2])
            if math.isnan(distance) or math.isinf(distance):
                self.nt.putValue('distance', -1)
                return c
            self.nt.putValue('distance', round(distance))
            return c
        self.nt.clearTable()

    def postProcess(self, frame, target):
        if target is -1:
            return frame
        drawnimage = cv2.drawContours(frame, [target], -1, (0, 255, 255), 2)
        return drawnimage