#!/usr/bin/env python

"""
Edge detection package
by Simone Silenzi version 0.1.0
"""

import rospy
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
        box = Box(idx=idx+1, width=dbox[0], length=dbox[1], height=dbox[2], rotmatrix=rotmatrix, center=center)
        boxesData.append(box)
    boxesData = tuple(boxesData)
    boxes = Boxes(len=len(boxesData), data=boxesData)
    return boxes


# init camera function
def initCameras(cams, caps):
    for cam in cams:
        # noinspection PyArgumentList
        cap = cv.VideoCapture(cam)
        if not cap.isOpened():
            for othercap in caps:
                othercap.release()
            raise IOError('Cannot open camera ' + str(cam))
        caps.append(cap)


def main():
    print(__doc__)
    rospy.init_node('edged')
    # init cameras
    cams = [2, 0]
    caps = []
    initCameras(cams, caps)
    # main loop
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        ret = False
        frames = []
        for cap in caps:
            ret, frame = cap.read()
            if not ret:
                break
            frames.append(frame)
        if ret:
            for itr, cam in enumerate(cams):
                cv.imshow('Camera ' + str(cam), frames[itr])
            rospy.loginfo('Printed images')
        c = cv.waitKey(1)
        if c != 255:
            break
        boxesIso = [[[434., 765., 65498.], [768., 345., 754.]], [[543., 976., 165.], [127., 985., 5987.]]]
        boxes = buildBoxes(boxesIso)
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
