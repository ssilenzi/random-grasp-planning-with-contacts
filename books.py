#!/usr/bin/env python

"""
Description
"""

import numpy as np
import cv2


def angle_cos(p0, p1, p2):
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def mean(p1, p2):
    return int((p1 + p2) / 2)


def find_squares(img):
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    # for idx, gray in enumerate(cv2.split(img)):
    for gray in cv2.split(img):
        for thrs in range(0, 255, 128):
            if thrs == 0:
                binary = cv2.Canny(gray, 0, 50, apertureSize=5)
                binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            else:
                _retval, binary = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            # cv2.imshow('bin_' + str(idx) + '_' + str(thrs), bin)
            contours, _hierarchy = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.005 * cnt_len, True)
                if any(min(point) < 2 or
                       point[0] > (img.shape[1] - 2) or
                       point[1] > (img.shape[0] - 2) for point in cnt[0]):
                    continue
                if len(cnt) >= 4 and cv2.contourArea(cnt) > 1000:  # and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos(cnt[i],
                                                cnt[(i + 1) % len(cnt)],
                                                cnt[(i + 2) % len(cnt)]) for i in range(len(cnt))])
                    if max_cos < 0.4:
                        squares.append(cnt)
    return squares


def main():
    from glob import glob
    for fn in glob('source/library2.jpg'):
        img = cv2.imread(fn)

        img2 = img.copy()

        squares = find_squares(img)
        cv2.drawContours(img, squares, -1, color=(0, 0, 255), thickness=1)

        approx = squares[0]
        n_boxes = int(len(approx) / 4)
        box = np.empty([n_boxes, 2, 2], dtype=int)
        for i in range(n_boxes):
            box[i] = [[mean(approx[1 + 2 * i, 0], approx[-2 - 2 * i, 0]),
                       mean(approx[2 * i, 1], approx[1 + 2 * i, 1])],
                      [mean(approx[2 * i, 0], approx[-1 - 2 * i, 0]),
                       mean(approx[-2 - 2 * i, 1], approx[-1 - 2 * i, 1])]]
            cv2.rectangle(img2, tuple(box[i, 0]), tuple(box[i, 1]), color=(255, 0, 0), thickness=1)
        cv2.imshow("rectangles", img2)
        cv2.imwrite("dest/rectangles.jpg", img2)

        cv2.imshow('squares', img)
        ch = cv2.waitKey()
        if ch == 27:
            break


if __name__ == '__main__':
    print(__doc__)
    main()
    cv2.destroyAllWindows()
