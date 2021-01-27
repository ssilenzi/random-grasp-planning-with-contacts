#!/usr/bin/env python

"""
Edge detection package
George Jose Pollayil and Simone Silenzi
version 0.1.0
"""

import cv2 as cv
import threading
import logging
import numpy as np
import rospkg
import rospy
from edge_detection.msg import Box, Boxes


class ThreadedCamera(object):
    def __init__(self, cam):
        # type: (int) -> None
        # noinspection PyArgumentList
        self.__cap = cv.VideoCapture(cam)
        self.__frame = np.array([])
        self.__stopCam = False
        self.__thread = threading.Thread(target=self.__update, args=())
        self.__thread.daemon = True
        self.__thread.start()

    def __update(self):
        # type: () -> None
        while True:
            if self.__cap.isOpened():
                _, self.__frame = self.__cap.read()
            if self.__stopCam:
                if self.__cap.isOpened():
                    self.__cap.release()
                break

    def grabFrame(self):
        # type: () -> np.ndarray
        return self.__frame

    def release(self):
        # type: () -> None
        self.__stopCam = True

    def isOpened(self):
        # type: () -> bool
        return self.__cap.isOpened()


def getParameters():
    # type: () -> (str, list, str, str, str, bool, bool, float, int, float, list, float, str, bool) or None
    (cams, file_front, file_top, path) = (None,) * 4
    in_mode = rospy.get_param('input', 'cameras')
    if in_mode == 'cameras':
        cams = rospy.get_param('cams', [0, 1])
        if len(cams) != 2:
            rospy.logfatal("edge_detection: you must specify two camera indexes in 'cameras' vector")
            return None
        rospy.loginfo('edge_detection: chosen cameras ' + str(cams))
    elif in_mode == 'files':
        if rospy.has_param('image_front') and rospy.get_param('image_top'):
            file_front = rospy.get_param('image_front')
            file_top = rospy.get_param('image_top')
            rospy.loginfo("edge_detection: using '" + file_front +
                          "' as front camera and '" + file_top + "' as top camera")
        else:
            rospy.logfatal("edge_detection: unspecified one or both filenames in parameters 'image_front' and "
                           "'image_top'")
            return None
    else:
        rospy.logfatal("edge_detection: 'input' can be only 'cameras' or 'files'")
        return None
    if rospy.has_param('first_mm'):
        first_mm = rospy.get_param('first_mm')
        first_list = [first_mm.get('width'), first_mm.get('length'), first_mm.get("height")]
        if any(measure is None for measure in first_list):
            rospy.logfatal("edge_detection: 'first_mm' is a dictionary with entries 'width', 'length' and 'height'")
            return None
        tmp_str = 'edge_detection: the first box has dimensions (mm): '
        for s in first_mm:
            tmp_str += s + ': ' + str(first_mm.get(s)) + ' mm, '
        tmp_str = tmp_str[0:-2]
        rospy.loginfo(tmp_str)
    else:
        rospy.logfatal("edge_detection: can't find the parameter 'first_mm' that contains the first box dimensions")
        return None
    rospack = rospkg.RosPack()
    path = rospack.get_path('edge_detection') + '/images/'
    multi_thresh = rospy.get_param('multi_thresh', False)
    only_ext = rospy.get_param('only_ext', True)
    epsilon = rospy.get_param('epsilon', 0.02)
    min_area = rospy.get_param('min_area', 5000)
    max_sin_cos = rospy.get_param('max_sin_cos', 0.5)
    rt = rospy.get_param('rate', 5)
    unit = rospy.get_param('unit', 'm')
    debug = rospy.get_param('debug', False)
    return (in_mode, cams, file_front, file_top, path, multi_thresh, only_ext, epsilon, min_area, max_sin_cos,
            first_list, rt, unit, debug)


def initCameras(cams, caps):
    # type: (list, list) -> None
    for cam in cams:
        # for every camera a capture thread is started
        # noinspection PyArgumentList
        cap = ThreadedCamera(cam)
        caps.append(cap)
        if not cap.isOpened():
            raise IOError


def loadImages(front, top):
    # type: (str, str) -> (np.ndarray, np.ndarray)
    img_front = cv.imread(front)
    if img_front is None:
        rospy.logfatal("edge_detection: image file '" + rospy.get_param('image_front') + "' not found")
        return (None,)*2
    img_top = cv.imread(top)
    if img_top is None:
        rospy.logfatal("edge_detection: image file '" + rospy.get_param('image_top') + "' not found")
        return (None,)*2
    return img_front, img_top


def mean(p1, p2):
    if isinstance(p1, int) and isinstance(p2, int):
        mean_value = int((p1 + p2) / 2)
    else:
        mean_value = (p1 + p2) / 2
    return mean_value


def angleCos(p0, p1, p2):
    # type: (np.ndarray, np.ndarray, np.ndarray) -> float
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def angleSin(p0, p1, p2):
    # type: (np.ndarray, np.ndarray, np.ndarray) -> float
    d1, d2 = (p0 - p1).astype('float'), (p2 - p1).astype('float')
    return abs(np.cross(d1, d2) / np.sqrt(np.dot(d1, d1) * np.dot(d2, d2)))


def findPolylines(img, multi_thresh, only_ext, epsilon, min_area, max_sin_cos):
    # type: (np.ndarray, bool, bool, float, int, float) -> (np.ndarray, list)
    blur = cv.GaussianBlur(img.copy(), (5,) * 2, 0)
    contours = []
    if multi_thresh:
        grays = cv.split(blur)
        thrss = xrange(0, 255, 63)
    else:
        grays = [blur]
        thrss = [0]
    if only_ext:
        method = cv.RETR_EXTERNAL
    else:
        method = cv.RETR_LIST
    for gray in grays:
        for thrs in thrss:
            if thrs == 0:
                sigma_ = np.std(gray)
                mean_ = np.mean(gray)
                lower = int(max(0, (mean_ - 2 * sigma_)))
                upper = int(min(255, (mean_ + 2 * sigma_)))
                binary = cv.Canny(gray, lower, upper, apertureSize=3)
                binary = cv.morphologyEx(binary, cv.MORPH_CLOSE, cv.getStructuringElement(cv.MORPH_RECT, (5,) * 2))
                canny = binary.copy()
            else:
                _, binary = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            tmp_contours, _ = cv.findContours(binary, method, cv.CHAIN_APPROX_SIMPLE)[-2:]
            contours.extend(tmp_contours)
    selected_contours = []
    origin = np.array([0, 0])
    i_vector = np.array([1, 0])
    j_vector = np.array([0, 1])
    if contours:
        for cnt in contours:
            cnt_len = cv.arcLength(cnt, True)
            # TODO depending on epsilon (i. e. 0.02) the contour changes shape significantly:
            #  is it a good idea trying to iterate on this number to extract a good contour? Maybe no
            cnt = cv.approxPolyDP(cnt, epsilon * cnt_len, True)
            # arrange contour's points as a matrix:
            #   every row is a point
            #   1st column -> x
            #   2nd column -> y
            cnt = cnt.reshape((-1, 2))
            # discard contours that are closer less than 2 pixels from borders
            if any(min(point) < 2 or
                   point[0] > (img.shape[1] - 2) or
                   point[1] > (img.shape[0] - 2) for point in cnt):
                continue
            # discard contours that have less than 4 sides or whose area is small
            if len(cnt) >= 4 and cv.contourArea(cnt) > min_area:
                # IMPORTANT assuming that boxes are approximately rectangles
                max_cos = max([angleCos(cnt[i], cnt[(i + 1) % len(cnt)], cnt[(i + 2) % len(cnt)])
                               for i in xrange(len(cnt))])
                if max_cos < max_sin_cos:
                    # IMPORTANT assuming that boxes are approximately vertical or horizontal
                    max_sin = max([min(angleSin(i_vector, origin, cnt[(i + 1) % len(cnt)] - cnt[i]),
                                       angleSin(j_vector, origin, cnt[(i + 1) % len(cnt)] - cnt[i]))
                                   for i in xrange(len(cnt))])
                    if max_sin < max_sin_cos:
                        selected_contours.append(cnt)
    return canny, selected_contours


def extractRects(contour, boxes_iso, axes):
    # type: (np.ndarray, np.ndarray, str) -> np.ndarray
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
        raise SyntaxError("extractRects accept only axes 'xy', 'xz', 'y' or 'z'")
    n_boxes = len(boxes_iso)
    boxes_plot = np.empty([n_boxes, 2, 2], dtype=int)
    # TODO Is this way to extract rectangles correct? Not using the x coordinate in any way.
    #  Are the points in the contour sorted always in the same manner?
    # IMPORTANT assuming that boxes are approximately vertical or horizontal
    for i in xrange(n_boxes):
        if ax1 is not None:
            boxes_iso[i, :, ax1] = np.array([mean(contour[1 + 2 * i, 0], contour[-2 - 2 * i, 0]),
                                             mean(contour[2 * i, 0], contour[-1 - 2 * i, 0])])
        else:
            ax1 = 0
        boxes_iso[i, :, ax2] = np.array([mean(contour[2 * i, 1], contour[1 + 2 * i, 1]),
                                         mean(contour[-2 - 2 * i, 1], contour[-1 - 2 * i, 1])])
        boxes_plot[i, :, 0] = boxes_iso[i, :, ax1]
        boxes_plot[i, :, 1] = boxes_iso[i, :, ax2]
    return boxes_plot


def tryToExtractRects(contours_front, contours_top):
    # type: (list, list) -> (bool, np.ndarray, np.ndarray, np.ndarray)
    # TODO Remove duplicates in contours -- not needed if multi_thresh = 0 -> only 1 color threshold
    # TODO Remove internal rectangles: RETR_EXTERNAL can help only with 1 color threshold
    boxes_detected = False
    boxes_iso_pxl = []
    boxes_plot_front = []
    boxes_plot_top = []
    if contours_front and contours_top:
        selected_indices = []
        n_max_contours = min(len(contours_front), len(contours_top))
        for selected_contour_front in contours_front:
            front_mult_4 = not (len(selected_contour_front) % 4)
            for idx, selected_contour_top in enumerate(contours_top):
                top_mult_4 = not (len(selected_contour_top) % 4)
                same_number = len(selected_contour_front) == len(selected_contour_top)
                not_selected = all(idx != sel for sel in selected_indices if selected_indices)
                if front_mult_4 and top_mult_4 and same_number and not_selected:
                    selected_indices.append(idx)
                    n_boxes = int(len(selected_contour_front) / 4)
                    # recognize rectangles in detected contours and store them in vectors
                    tmp_boxes_iso = np.empty([n_boxes, 2, 3], dtype=int)
                    boxes_plot_front.extend(extractRects(selected_contour_front, tmp_boxes_iso, 'xz'))
                    boxes_plot_top.extend(extractRects(selected_contour_top, tmp_boxes_iso, 'y'))
                    boxes_iso_pxl.extend(tmp_boxes_iso)
                    boxes_detected = True
                    if len(selected_indices) >= n_max_contours:
                        break
            if len(selected_indices) >= n_max_contours:
                break
    if boxes_detected:
        boxes_iso_pxl = np.array(boxes_iso_pxl)
        boxes_plot_front = np.array(boxes_plot_front)
        boxes_plot_top = np.array(boxes_plot_top)
    return boxes_detected, boxes_iso_pxl, boxes_plot_front, boxes_plot_top


def scaleBoxes(boxes_iso_pxl, first_list_mm, unit):
    # type: (np.ndarray, list, str) -> np.ndarray or None
    idx = boxes_iso_pxl[:, 0, 0].argmin()
    first_pxl = boxes_iso_pxl[idx, 1, :] - boxes_iso_pxl[idx, 0, :]
    if any(num == 0 for num in first_pxl):
        return None
    scale_factors = np.divide(first_list_mm, first_pxl.astype('float'))
    if unit == 'm':
        scale_factors = 0.001 * scale_factors
    elif unit == 'mm':
        pass
    else:
        raise SyntaxError("scaleBoxes accepts only 'm' (default) or 'mm'")
    scale_factors_mat = np.diag(scale_factors)
    origin_mat_pxl = np.tile(boxes_iso_pxl[idx, 0, :], (len(boxes_iso_pxl), 2, 1))
    boxes_iso_pxl_rel = boxes_iso_pxl - origin_mat_pxl
    boxes_iso = np.empty(boxes_iso_pxl_rel.shape, dtype=float)
    for idx, box_iso in enumerate(boxes_iso_pxl_rel):
        boxes_iso[idx, :, :] = np.matmul(box_iso, scale_factors_mat)
    return boxes_iso


def buildBoxes(boxes_iso):
    # type: (np.ndarray) -> Boxes
    boxes_data = []
    for idx, box_iso in enumerate(boxes_iso):
        box_iso = np.array(box_iso).astype('float')
        dbox = box_iso[1] - box_iso[0]
        dbox = tuple(dbox.tolist())
        rot_matrix = np.identity(3)
        rot_matrix = tuple(rot_matrix.flatten().tolist())
        center = tuple(mean(box_iso[0], box_iso[1]))
        box = Box(idx=idx + 1, width=dbox[0], length=dbox[1], height=dbox[2], rot_matrix=rot_matrix, center=center)
        boxes_data.append(box)
    boxes_data = tuple(boxes_data)
    boxes = Boxes(len=len(boxes_data), data=boxes_data)
    return boxes


def checkKeyboard():
    # type: () -> bool
    # check if the user wants to terminate the program
    key = cv.waitKey(1)
    if key != 255:
        return True
    # the user doesn't want to terminate -> go ahead with the next iteration
    return False


def main():
    print(__doc__)
    # init node
    rospy.init_node('edge_detection', anonymous=True)
    # publish data into private topic 'boxes'
    pub = rospy.Publisher('boxes', Boxes, queue_size=1000)
    # get parameters from file
    tmp_tup = getParameters()
    if tmp_tup is None:
        return
    else:
        (in_mode, cams, file_front, file_top, path, multi_thresh, only_ext, epsilon, min_area, max_sin_cos,
         first_list_mm, rt, unit, debug) = tmp_tup
    # set proper logger level
    if debug:
        logger = logging.getLogger("rosout")
        logger.setLevel(logging.DEBUG)
    # init cameras
    caps = []
    if in_mode == 'cameras':
        try:
            initCameras(cams, caps)
        except IOError:
            for idx, cap in enumerate(caps):
                if not cap.isOpened():
                    cam = cams[idx]
                    rospy.logfatal("edge_detection: can't open camera " + str(cam))
                else:
                    cap.release()
            return

    # main loop
    rate = rospy.Rate(rt)  # Hz
    while not rospy.is_shutdown():
        # grab actual frames from all cameras -- 'cameras' mode
        if in_mode == 'cameras':
            frames = []
            for cap in caps:
                frames.append(cap.grabFrame())
                if not frames[-1].size:
                    break
            # check if we received the images from cameras
            if frames[-1].size:
                # cameras have images
                img_front = frames[0]
                img_top = frames[1]
            else:
                # cameras don't have images
                if checkKeyboard():
                    break
                continue
        # otherwise load images from files -- 'files' mode
        else:
            img_front, img_top = loadImages(path + file_front, path + file_top)
            if (img_front is None) or (img_top is None):
                return
        # find contours in both images and store them in vectors
        binary_front, contours_front = findPolylines(img_front, multi_thresh, only_ext, epsilon, min_area, max_sin_cos)
        binary_top, contours_top = findPolylines(img_top, multi_thresh, only_ext, epsilon, min_area, max_sin_cos)
        (boxes_detected, boxes_iso_pxl, boxes_plot_front, boxes_plot_top) = \
            tryToExtractRects(contours_front, contours_top)
        # publish nothing if the number of sides of contours don't match
        if boxes_detected:
            # convert from pixels to meters and take local coordinates (from the first box)
            boxes_iso = scaleBoxes(boxes_iso_pxl, first_list_mm, unit)
            if boxes_iso is None:
                rospy.logwarn('The first box measures 0 pxl in one side -> ignore')
                continue
            # create the boxes structure and publish data
            boxes = buildBoxes(boxes_iso)
            pub.publish(boxes)
        else:
            rospy.loginfo('edge_detection: none of the contours in the first image matches the contours in the second '
                          'image')
        if boxes_detected:
            img_boxes_front = img_front.copy()
            img_boxes_top = img_top.copy()
        if contours_front:
            cv.drawContours(img_front, contours_front, -1, color=(0, 0, 255), thickness=3)
        if contours_top:
            cv.drawContours(img_top, contours_top, -1, color=(0, 0, 255), thickness=3)
        if boxes_detected:
            for rect in boxes_plot_front:
                cv.rectangle(img_boxes_front, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=3)
            for rect in boxes_plot_top:
                cv.rectangle(img_boxes_top, tuple(rect[0]), tuple(rect[1]), color=(255, 0, 0), thickness=3)
        cv.imshow('Front camera', img_front)
        cv.imshow('Top camera', img_top)
        cv.imshow('Front canny', binary_front)
        cv.imshow('Top canny', binary_top)
        if boxes_detected:
            cv.imshow("Front boxes", img_boxes_front)
            cv.imshow("Top boxes", img_boxes_top)
        if checkKeyboard():
            break
        rospy.logdebug('edge_detection: task completed')
        rate.sleep()

    # close the program
    for cap in caps:
        cap.release()
    cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
