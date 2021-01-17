#!/usr/bin/env python

"""
Description
"""

import cv2 as cv


def main():
    from sys import platform
    if platform == "win32":
        cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    else:
        cap = cv.VideoCapture(1)
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    while True:
        ret, frame = cap.read()
        cv.imshow('Input', frame)

        c = cv.waitKey(1)
        if c == 27:
            break
    cap.release()


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
