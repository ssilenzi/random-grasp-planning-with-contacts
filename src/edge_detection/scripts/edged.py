#!/usr/bin/env python

"""
It detects boxes
"""

import cv2 as cv
import rospy
# from edge_detection.msg import Box


# noinspection PyArgumentList
def main():
    print(__doc__)
    rospy.init_node('edge_detector')
    # init first camera
    cam1 = 2
    cap1 = cv.VideoCapture(cam1)
    if not cap1.isOpened():
        raise IOError('Cannot open camera ' + str(cam1))
    # init second camera
    cam2 = 0
    cap2 = cv.VideoCapture(cam2)
    if not cap2.isOpened():
        cap1.release()
        raise IOError('Cannot open camera ' + str(cam2))
    # main loop
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        ret, frame1 = cap1.read()
        ret, frame2 = cap2.read()
        cv.imshow('Camera ' + str(cam1), frame1)
        cv.imshow('Camera ' + str(cam2), frame2)
        c = cv.waitKey(1)
        if c != 255:
            break
        rospy.loginfo("Showed images")
        rate.sleep()
    # close program
    cap1.release()
    cap2.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
