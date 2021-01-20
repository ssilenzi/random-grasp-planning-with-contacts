#!/usr/bin/env python

"""
Description
"""

import numpy as np
import cv2 as cv


class Box:
    def __init__(self, width, length, height, ref_frame_T: np.ndarray):
        self.l = length
        self.w = width
        self.h = height
        self.T = ref_frame_T


def mean(p1, p2):
    if isinstance(p1, int) and isinstance(p2, int):
        mean_value = int((p1 + p2) / 2)
    else:
        mean_value = (p1 + p2) / 2
    return mean_value


def build_boxes(boxes_iso):
    boxes = []
    for box in boxes_iso:
        T = np.identity(4)
        T[0:3, 3] = mean(box[0], box[1])
        df = box[1] - box[0]
        boxes.append(Box(width=df[0], length=df[1], height=df[2], ref_frame_T=T))
    return boxes


def angle_cos(p0, p1, p2):
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def find_squares(img):
    img = cv.GaussianBlur(img, (5, 5), 0)
    selected_contours = []
    for gray in cv.split(img):
        for thrs in range(0, 255, 127):
            if thrs == 0:
                binary = cv.Canny(gray, 0, 50, apertureSize=5)
                binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (3, 3)))
            else:
                _retval, binary = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            contours, _hierarchy = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
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
                        # TODO Remove duplicates in contours
                        selected_contours.append(cnt)
    return selected_contours


def extract_rects(contour, boxes_iso, axes):
    ax1 = None
    if axes == 'xy':
        ax1 = 0
        ax2 = 1
    elif axes == 'xz':
        ax1 = 0
        ax2 = 2
    elif axes == 'y':
        ax2 = 1
    elif axes == 'z':
        ax2 = 2
    else:
        raise SyntaxError("extract_rects accept only axes 'xy', 'xz', 'y' or 'z'")
    n_boxes = boxes_iso.shape[0]
    boxes_plot = np.empty([n_boxes, 2, 2], dtype=int)
    for i in range(n_boxes):
        if ax1 is not None:
            boxes_iso[i, :, ax1] = [mean(contour[1 + 2 * i, 0], contour[-2 - 2 * i, 0]),
                                    mean(contour[2 * i, 0], contour[-1 - 2 * i, 0])]
        else:
            ax1 = 0
        boxes_iso[i, :, ax2] = [mean(contour[2 * i, 1], contour[1 + 2 * i, 1]),
                                mean(contour[-2 - 2 * i, 1], contour[-1 - 2 * i, 1])]
        boxes_plot[i, :, 0] = boxes_iso[i, :, ax1]
        boxes_plot[i, :, 1] = boxes_iso[i, :, ax2]
    return boxes_plot


def main():
    # TODO Acquire camera and create the while loop
    fn = 'img_src/library.jpg'
    img_front = cv.imread(fn)
    img_top = img_front.copy()
    img_front2 = img_front.copy()
    img_top2 = img_top.copy()

    contours_front = find_squares(img_front)
    cv.drawContours(img_front, contours_front, -1, color=(0, 0, 255), thickness=1)
    cv.imshow('Front profile', img_front)

    contours_top = find_squares(img_top)
    cv.drawContours(img_top, contours_top, -1, color=(0, 0, 255), thickness=1)
    cv.imshow('Top profile', img_top)

    selected_contour_front = contours_front[0]
    selected_contour_top = contours_top[0]
    # TODO Continue the loop if dimensions of contours are wrong
    n_boxes = int(len(selected_contour_front) / 4)
    # TODO Calculate aspect ratios of axes x, y, z

    boxes_iso_pxl = np.empty([n_boxes, 2, 3], dtype=int)
    boxes_plot = extract_rects(selected_contour_front, boxes_iso_pxl, 'xz')
    for rect in boxes_plot:
        cv.rectangle(img_front2, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=1)
    cv.imshow("Front boxes", img_front2)

    boxes_plot = extract_rects(selected_contour_top, boxes_iso_pxl, 'y')
    for rect in boxes_plot:
        cv.rectangle(img_top2, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=1)
    cv.imshow("Top boxes", img_top2)

    #TODO Create boxes_iso using aspect ratios
    boxes_iso = boxes_iso_pxl.copy()
    boxes = build_boxes(boxes_iso)

    cv.waitKey()


if __name__ == '__main__':
    print(__doc__)
    main()
    cv.destroyAllWindows()
