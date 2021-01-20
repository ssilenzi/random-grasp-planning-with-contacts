#!/usr/bin/env python

"""
It detects books
"""

import cv2 as cv


def main():
    cap = cv.VideoCapture(2)
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
