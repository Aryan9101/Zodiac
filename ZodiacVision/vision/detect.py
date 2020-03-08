import cv2
import numpy as np
import yaml
import pyzed.sl as sl
import math


class Detector:
    def __init__(self, nt):
        self.frame = 0
        self.nt = nt

    def proc_part(self, part):
        contours2, hier2 = cv2.findContours(part, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours2) > 0:
            cont2 = contours2[0]

            rect = cv2.minAreaRect(cont2)
            (x1, y1), (x2, y2), angle = rect
            xa = x1 + x2 / 2

            # angle = math.atan2(y1 - y2, x1-x2)
            # math.degrees(angle)
            # print(f"RECT: {rect}")
            print(angle)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            # print(box)
            return box, angle

        return False, False
    def preProcessFrame(self, frame):
        lower = self.nt.yml_data['color']['lower']
        upper = self.nt.yml_data['color']['upper']
        # Preprocess
        lower_color = (lower['H'], lower['S'], lower['V'])
        upper_color = (upper['H'], upper['S'], upper['V'])
        h, w, _ = frame.shape
        image = frame[0:int(h / 2), 0:w]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_color, upper_color)
        return mask
    def findTarget(self, img, zed, point_cloud):
        oldImg = img.copy()
        contours, hierarchy = cv2.findContours(oldImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        hexes = []
        # debug code to show some contours for frame 100
        # print(f"Got contours, count: {len(contours)}")
        # print(contours)
        # img = cv2.drawContours(img, contours, -1, (0,255,255), 2)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        top_3_contours = []
        if len(cntsSorted) > 0:
            top_3_contours.append(cntsSorted[0])
        # if len(cntsSorted) > 1:
        #     top_3_contours.append(cntsSorted[1])
        # if len(cntsSorted) > 2:
        #     top_3_contours.append(cntsSorted[2])
        for cont in top_3_contours:
            x, y, w, h = cv2.boundingRect(cont)
            # only process larger areas with at least 5 points in the contour
            if len(cont) > 4:
                quarter_w = int(w * 0.25)
                x0 = x
                y0 = y
                x1 = x + quarter_w
                y1 = y + h
                x2 = x + w - quarter_w
                x3 = x + w
                #
                # cv2.rectangle(img, (x0, y0), (x1, y1), (255, 255, 0), 2)
                # cv2.rectangle(img, (x1, y0), (x2, y1), (255, 255, 0), 2)
                # cv2.rectangle(img, (x2, y0), (x3, y1), (255, 255, 0), 2)

                part1 = img[y0:y1, x0:x1]
                print('RECT1')
                box1, angle1 = self.proc_part(part1)
                # x1 + x2
                if box1 is False or angle1 > -5 or angle1 < -40:
                    continue
                box1a = box1 + [x0, y0]
                # print(f"{x0:3d}/{y0:3d}  box1: {box1a}")
                # img = cv2.drawContours(img, [box1a], -1, (0, 0, 255), 2)

                part2 = img[y0:y1, x1:x2]
                # print('RECT2')
                box2, angle2 = self.proc_part(part2)
                if box2 is False or -75 < angle2 < -30:
                    continue
                box2a = box2 + [x1, y0]
                # print(f"box2: {box2a}")
                # img = cv2.drawContours(img, [box2a], -1, (0, 0, 255), 2)

                part3 = img[y0:y1, x2:x3]
                print('RECT3')
                box3, angle3 = self.proc_part(part3)

                if box3 is False or angle3 > -30:
                    continue

                box3a = box3 + [x2, y0]
                print(box1a)
                centerX = int((box3a[2][0] + box1a[2][0]) / 2)
                centerY = int((box3a[3][1] + box1a[2][1]) / 2)
                # print(f"box3: {box3a}")
                # img = cv2.drawContours(img, [box3a], -1, (0, 0, 255), 2)

                hexes.append(cont)
                img = cv2.line(img, (centerX, centerY), (centerX, centerY), (255, 255, 0), 5)
                img = cv2.rectangle(img, (box1a[1][0], box1a[0][1]), (box3a[3][0], int((box3a[2][1] / 2))),
                                    (255, 255, 0), 2)
                # imgx = cv2.resize(img[y:y+h, x:x+w], None, fx=4.0, fy=4.0)
                # cv2.imshow('imgx', imgx)
            # else:
            #     print(f"ELSE: bounding rect x y w h: {x:3d} {y:3d} {w:3d} {h:3d}")
        # print(len(hexes))

        # img = cv2.drawContours(img, hexes, -1, (0, 0, 255), 1)
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img
