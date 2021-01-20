#!/usr/bin/env python

"""
Edge detection package
by Simone Silenzi version 0.1.0
"""

import rospy
from edge_detection.msg import Box
import cv2 as cv
import numpy as np


# init camera function
def initCameras(cams, caps):
    for cam in cams:
        # noinspection PyArgumentList
        cap = cv.VideoCapture(cam)
        if not cap.isOpened():
            for ocap in caps:
                ocap.release()
            raise IOError('Cannot open camera ' + str(cam))
        caps.append(cap)


def main():
    print(__doc__)
    rospy.init_node('edge_detector')
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
            for i, cam in enumerate(cams):
                cv.imshow('Camera ' + str(cam), frames[i])
            rospy.loginfo("Printed images")
        c = cv.waitKey(1)
        if c != 255:
            break
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
