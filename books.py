#!/usr/bin/env python

"""
Description
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
    selectedContours = []
    # for idx, gray in enumerate(cv.split(img)):
    for gray in cv.split(img):
        for thrs in range(0, 255, 128):
            if thrs == 0:
                binary = cv.Canny(gray, 0, 50, apertureSize=5)
                binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (3, 3)))
            else:
                _retval, binary = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            # cv.imshow('bin_' + str(idx) + '_' + str(thrs), bin)
            contours, _hierarchy = cv.findContours(binary, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
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
                        selectedContours.append(cnt)
    return selectedContours


def main():
    from glob import glob
    for fn in glob('source/library.jpg'):
        imgTop = cv.imread(fn)
        imgFront = cv.imread(fn)
        imgTop2 = imgTop.copy()
        imgFront2 = imgFront.copy()

        selectedContours = find_squares(imgTop)
        cv.drawContours(imgTop, selectedContours, -1, color=(0, 0, 255), thickness=1)
        contourTop = selectedContours[0]
        cv.imshow('Top profile', imgTop)
        # cv.imshow('Front profile', imgFront)

        nBoxes = int(len(contourTop) / 4)
        box = np.empty([nBoxes, 2, 2], dtype=int)
        for i in range(nBoxes):
            box[i] = [[mean(contourTop[1 + 2 * i, 0], contourTop[-2 - 2 * i, 0]),
                       mean(contourTop[2 * i, 1], contourTop[1 + 2 * i, 1])],
                      [mean(contourTop[2 * i, 0], contourTop[-1 - 2 * i, 0]),
                       mean(contourTop[-2 - 2 * i, 1], contourTop[-1 - 2 * i, 1])]]
            cv.rectangle(imgTop2, tuple(box[i, 0]), tuple(box[i, 1]), color=(255, 0, 0), thickness=1)
        cv.imshow("Top boxes", imgTop2)
        cv.imwrite("dest/top_boxes.jpg", imgTop2)

        ch = cv.waitKey()
        if ch == 27:
            break


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
