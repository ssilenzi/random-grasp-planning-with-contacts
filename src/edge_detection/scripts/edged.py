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


def mean(p1, p2):
    if isinstance(p1, int) and isinstance(p2, int):
        meanValue = int((p1 + p2) / 2)
    else:
        meanValue = (p1 + p2) / 2
    return meanValue


def buildBoxes(boxesIso):
    boxesData = []
    for idx, boxIso in enumerate(boxesIso):
        boxIso = np.array(boxIso)
        dbox = boxIso[1] - boxIso[0]
        dbox = tuple(dbox.tolist())
        rotmatrix = np.identity(3)
        rotmatrix = tuple(rotmatrix.flatten().tolist())
        center = tuple(mean(boxIso[0], boxIso[1]))
        box = Box(idx=idx + 1, width=dbox[0], length=dbox[1], height=dbox[2], rotmatrix=rotmatrix, center=center)
        boxesData.append(box)
    boxesData = tuple(boxesData)
    boxes = Boxes(len=len(boxesData), data=boxesData)
    return boxes


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
                (self.__ret, self.__frame) = self.__cap.read()
            if self.__stopCam:
                if self.__cap.isOpened():
                    self.__cap.release()
                break

    def grabFrame(self):
        if self.__ret:
            return self.__frame
        return None

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
            for otherCap in caps:
                otherCap.release()
            raise IOError('Cannot open camera ' + str(cam))
        caps.append(cap)


def main():
    print(__doc__)
    pub = rospy.Publisher('boxes', Boxes, queue_size=1000)
    rospy.init_node('edge_detection', anonymous=True)
    # init cameras
    cams = [2, 0]
    caps = []
    initCameras(cams, caps)
    # main loop
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        # grab actual frame from all cameras
        frames = []
        frame = None
        for cap in caps:
            frame = cap.grabFrame()
            if frame is None:
                break
            frames.append(frame)
        # publish data into private topic 'boxes'
        rospy.loginfo('Recognized objects')
        boxesIso = [[[434., 765., 65498.], [768., 345., 754.]],
                    [[543., 976., 165.], [127., 985., 5987.]]]  # an example
        boxes = buildBoxes(boxesIso)
        pub.publish(boxes)
        # display the results to user in some windows
        if frame is not None:
            for i, cam in enumerate(cams):
                cv.imshow('Camera ' + str(cam), frames[i])
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
