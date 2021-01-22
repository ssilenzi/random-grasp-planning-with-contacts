#!/usr/bin/env python

"""
Edge detection package
by Simone Silenzi version 0.1.0
"""

import rospy
import threading
from edge_detection.msg import Box, Boxes
import cv2 as cv
import numpy as np


class ThreadedCamera(object):
    def __init__(self, cam=0):
        self.__ret = False
        self.__frame = None
        self.__stopCam = False
        # noinspection PyArgumentList
        self.__cap = cv.VideoCapture(cam)
        self.__thread = threading.Thread(target=self.__update, args=())
        self.__thread.daemon = True
        self.__thread.start()

    def __update(self):
        while True:
            if self.__cap.isOpened():
                self.__ret, self.__frame = self.__cap.read()
            if self.__stopCam:
                if self.__cap.isOpened():
                    self.__cap.release()
                break

    def grabFrame(self):
        if self.__ret:
            return True, self.__frame
        return False, None

    def release(self):
        self.__stopCam = True

    def isOpened(self):
        return self.__cap.isOpened()


# init camera function
def initCameras(cams, caps):
    for cam in cams:
        # for every camera a capture thread is started
        # noinspection PyArgumentList
        cap = ThreadedCamera(cam)
        if not cap.isOpened():
            for other_cap in caps:
                other_cap.release()
            raise IOError('Cannot open camera ' + str(cam))
        caps.append(cap)


def mean(p1, p2):
    if isinstance(p1, int) and isinstance(p2, int):
        mean_value = int((p1 + p2) / 2)
    else:
        mean_value = (p1 + p2) / 2
    return mean_value


def angleCos(p0, p1, p2):
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def findPolylines(img):
    img = cv.GaussianBlur(img, (5, 5), 0)
    selected_contours = []
    for gray in cv.split(img):
        for thrs in xrange(0, 255, 127):
            if thrs == 0:
                binary = cv.Canny(gray, 0, 50, apertureSize=5)
                binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (3, 3)))
            else:
                _ret, binary = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            contours, _hierarchy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
            for cnt in contours:
                cnt_len = cv.arcLength(cnt, True)
                cnt = cv.approxPolyDP(cnt, 0.005 * cnt_len, True)
                # arrange contour's points as a matrix:
                #   every row is a point
                #   1st column -> x
                #   2nd column -> y
                cnt = cnt.reshape(-1, 2)
                # discard contours that are closer less than 2 pixels from borders
                if any(min(point) < 2 or
                       point[0] > (img.shape[1] - 2) or
                       point[1] > (img.shape[0] - 2) for point in cnt):
                    continue
                # discard contours that have less than 4 sides or whose area is small
                if len(cnt) >= 4 and cv.contourArea(cnt) > 1000:
                    # drop contours which have angles between 2 sides whose cosine is less than an amount
                    max_cos = np.max([angleCos(cnt[i],
                                               cnt[(i + 1) % len(cnt)],
                                               cnt[(i + 2) % len(cnt)]) for i in xrange(len(cnt))])
                    if max_cos < 0.4:
                        selected_contours.append(cnt)
    return selected_contours


def extractRects(contour, boxes_iso, axes):
    ax1 = None
    if axes == 'xy':
        ax1 = 0
        ax2 = 1
    elif axes == 'xz':
        ax1 = 0
        ax2 = 2
    elif axes == 'y':
        ax2 = 1
    elif axes == 'z':
        ax2 = 2
    else:
        raise SyntaxError("extractRects accept only axes 'xy', 'xz', 'y' or 'z'")
    n_boxes = len(boxes_iso)
    boxes_plot = np.empty([n_boxes, 2, 2], dtype=int)
    for i in xrange(n_boxes):
        if ax1 is not None:
            boxes_iso[i, :, ax1] = [mean(contour[1 + 2 * i, 0], contour[-2 - 2 * i, 0]),
                                    mean(contour[2 * i, 0], contour[-1 - 2 * i, 0])]
        else:
            ax1 = 0
        boxes_iso[i, :, ax2] = [mean(contour[2 * i, 1], contour[1 + 2 * i, 1]),
                                mean(contour[-2 - 2 * i, 1], contour[-1 - 2 * i, 1])]
        boxes_plot[i, :, 0] = boxes_iso[i, :, ax1]
        boxes_plot[i, :, 1] = boxes_iso[i, :, ax2]
    return boxes_plot


def buildBoxes(boxes_iso):
    boxes_data = []
    for idx, box_iso in enumerate(boxes_iso):
        box_iso = np.array(box_iso).astype('float')
        dbox = box_iso[1] - box_iso[0]
        dbox = tuple(dbox.tolist())
        rot_matrix = np.identity(3)
        rot_matrix = tuple(rot_matrix.flatten().tolist())
        center = tuple(mean(box_iso[0], box_iso[1]))
        box = Box(idx=idx + 1, width=dbox[0], length=dbox[1], height=dbox[2], rot_matrix=rot_matrix, center=center)
        boxes_data.append(box)
    boxes_data = tuple(boxes_data)
    boxes = Boxes(len=len(boxes_data), data=boxes_data)
    return boxes


def main():
    print(__doc__)
    # init node
    rospy.init_node('edge_detection', anonymous=True)
    # publish data into private topic 'boxes'
    pub = rospy.Publisher('boxes', Boxes, queue_size=1000)
    # init cameras:
    #   first index is the front camera (xz)
    #   second index is the top camera (xy)
    cams = [2, 0]
    caps = []
    initCameras(cams, caps)

    # main loop
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        # grab actual frames from all cameras
        ret = False
        frames = []
        for cap in caps:
            ret, frame = cap.grabFrame()
            if not ret:
                break
            frames.append(frame)
        if not ret:
            continue
        # TEMP import from file -- remove this block of code
        fn = '/media/simone/DATA/Users/Simone/Documents/linux-workspace/ROS/img_src/library.jpg'
        img_front = cv.imread(fn)
        img_top = img_front.copy()
        # find contours in both images and store them in vectors
        contours_front = findPolylines(img_front)
        contours_top = findPolylines(img_top)
        # TODO Remove duplicates in contours
        selected_contour_front = contours_front[0]
        selected_contour_top = contours_top[0]
        # recognize rectangles in detected contours and store them in vectors
        # TODO Continue the loop if dimensions of contours are wrong
        n_boxes = int(len(selected_contour_front) / 4)
        boxes_iso_pxl = np.empty([n_boxes, 2, 3], dtype=int)
        boxes_plot_cam1 = extractRects(selected_contour_front, boxes_iso_pxl, 'xz')
        boxes_plot_cam2 = extractRects(selected_contour_top, boxes_iso_pxl, 'y')
        # TODO Calculate aspect ratios of axes x, y, z and create boxes_iso using aspect ratios
        boxes_iso = boxes_iso_pxl.copy()
        # create the boxes structure and publish data
        boxes = buildBoxes(boxes_iso)
        pub.publish(boxes)
        rospy.logdebug('Timing')
        # display the results to user in some windows
        # img_front = frames[0].copy()
        # img_top = frames[1].copy()
        img_front2 = img_front.copy()
        img_top2 = img_top.copy()
        cv.drawContours(img_front, contours_front, -1, color=(0, 0, 255), thickness=1)
        cv.drawContours(img_top, contours_top, -1, color=(0, 0, 255), thickness=1)
        for rect in boxes_plot_cam1:
            cv.rectangle(img_front2, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=1)
        for rect in boxes_plot_cam2:
            cv.rectangle(img_top2, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=1)
        cv.imshow('Front camera', frames[0])
        cv.imshow('Top camera', frames[1])
        cv.imshow('Front profile', img_front)
        cv.imshow('Top profile', img_top)
        cv.imshow("Front boxes", img_front2)
        cv.imshow("Top boxes", img_top2)
        # check if the user wants to terminate the program
        key = cv.waitKey(1)
        if key != 255:
            break
        # sleep
        rate.sleep()

    # close the program
    for cap in caps:
        cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
