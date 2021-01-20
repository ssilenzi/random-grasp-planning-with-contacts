#!/usr/bin/env python

"""
Edge detection package
by S. Silenzi version 0.1.0
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


# node main program
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
        ret, frame1 = caps[0].read()
        ret, frame2 = caps[1].read()
        cv.imshow('Camera ' + str(cams[0]), frame1)
        cv.imshow('Camera ' + str(cams[1]), frame2)
        c = cv.waitKey(1)
        if c != 255:
            break
        rospy.loginfo("Showed images")
        rate.sleep()
    # close program
    caps[0].release()
    caps[1].release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
