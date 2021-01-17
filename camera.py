#!/usr/bin/env python

"""
Description
"""

import cv2


def main():
    from sys import platform
    if platform == "win32":
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(1)
    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    while True:
        ret, frame = cap.read()
        cv2.imshow('Input', frame)

        c = cv2.waitKey(1)
        if c == 27:
            break
    cap.release()


if __name__ == '__main__':
    print(__doc__)
    main()
    cv2.destroyAllWindows()
