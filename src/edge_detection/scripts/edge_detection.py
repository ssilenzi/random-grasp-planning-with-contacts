#!/usr/bin/env python

"""
It detects boxes
"""

import rospy
import cv2 as cv
# from edge_detection.msg import Box


# noinspection PyArgumentList
def main():
    rospy.init_node('edge_detector')
    cam_n = 2
    cap = cv.VideoCapture(cam_n)
    if not cap.isOpened():
        raise IOError('Cannot open camera ' + str(cam_n))
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        cv.imshow('Camera ' + str(cam_n), frame)
        cv.waitKey(1)
        rospy.loginfo("Showed image")
        rate.sleep()
    cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        print(__doc__)
        main()
    except rospy.ROSInterruptException:
        pass
