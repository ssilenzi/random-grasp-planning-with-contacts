#!/usr/bin/env python

"""
Simple "Square Detector" program.

Loads several images sequentially and tries to find squares in each image.
"""

import numpy as np
import cv2 as cv


def angle_cos(p0, p1, p2):
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def mean(p1, p2):
    return int((p1 + p2) / 2)


def find_squares(img):
    img = cv.GaussianBlur(img, (5, 5), 0)
    squares = []
    # for idx, gray in enumerate(cv.split(img)):
    for gray in cv.split(img):
        for thrs in range(0, 255, 128):
            if thrs == 0:
                bin = cv.Canny(gray, 0, 50, apertureSize=5)
                bin = cv.morphologyEx(bin, cv.MORPH_CLOSE, kernel=cv.getStructuringElement(cv.MORPH_RECT, (3, 3)))
            else:
                _retval, bin = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            # cv.imshow('bin_' + str(idx) + '_' + str(thrs), bin)
            contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv.arcLength(cnt, True)
                cnt = cv.approxPolyDP(cnt, 0.005 * cnt_len, True)
                if any(min(point) < 2 or
                       point[0] > (img.shape[1] - 2) or
                       point[1] > (img.shape[0] - 2) for point in cnt[0]):
                    continue
                if len(cnt) >= 4 and cv.contourArea(cnt) > 1000:  # and cv.isContourConvex(cnt):
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
        img = cv.imread(fn)

        img2 = img.copy()

        squares = find_squares(img)
        cv.drawContours(img, squares, -1, color=(0, 0, 255), thickness=1)

        approx = squares[0]
        n_boxes = int(len(approx) / 4)
        box = np.empty([n_boxes, 2, 2], dtype=int)
        for i in range(n_boxes):
            box[i] = [[mean(approx[1 + 2 * i, 0], approx[-2 - 2 * i, 0]),
                       mean(approx[2 * i, 1], approx[1 + 2 * i, 1])],
                      [mean(approx[2 * i, 0], approx[-1 - 2 * i, 0]),
                       mean(approx[-2 - 2 * i, 1], approx[-1 - 2 * i, 1])]]
            cv.rectangle(img2, tuple(box[i, 0]), tuple(box[i, 1]), color=(255, 0, 0), thickness=1)
        cv.imshow("rectangles", img2)
        cv.imwrite("dest/rectangles.jpg", img2)

        cv.imshow('squares', img)
        ch = cv.waitKey()
        if ch == 27:
            break
    print('Done')


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
