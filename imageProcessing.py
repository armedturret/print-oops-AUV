import numpy as np # numerical functions
import matplotlib.pyplot as plt # plotting tools
import cv2 # opencv

class imageProcessing():
    def __init__(self):
        self.__image = None

    def __detect_green_buoy(self, img):
        #img = cv2.imread(file_name)
        img = cv2.resize(img, (640, 480))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imhsv = cv2.boxFilter(img, -1, (10,10))
        img_thresh_hue = np.logical_and( imhsv[:,:,0] > 40, imhsv[:,:,0] < 80)
        img_thresh_sat = np.logical_and( imhsv[:,:,1] > 100, imhsv[:,:,1] < 160)
        img_thresh_val = np.logical_and( imhsv[:,:,2] > 210, imhsv[:,:,2] < 250) 
        img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)

        centers = np.argwhere(img_thresh_HSV > 50)
        center = np.mean(centers, axis=0) if centers.shape[0] > 0 else np.array([])
        
        return center

    def __detect_red_buoy(self, img):
        img = cv2.resize(img, (640, 480))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imhsv = cv2.boxFilter(img, -1, (10,10))
        img_thresh_hue = np.logical_and( imhsv[:,:,0] > 110, imhsv[:,:,0] < 170)
        img_thresh_sat = np.logical_and( imhsv[:,:,1] > 90, imhsv[:,:,1] < 130)
        img_thresh_val = np.logical_and( imhsv[:,:,2] > 220, imhsv[:,:,2] < 250)
        img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)

        centers = np.argwhere(img_thresh_HSV > 50)
        center = np.mean(centers, axis=0) if centers.shape[0] > 0 else np.array([])
        
        return center

    def __sensor_position(pix_x, res_x): 
        sensor_pos_x = (pix_x - (res_x / 2.0)) / res_x * 3.68

        return sensor_pos_x

    def __sensor_angles(pos_x):
        focal_length = 3.04

        horizontal_angle = np.arctan2(pos_x,focal_length)
        horizontal_angle = np.degrees(horizontal_angle)
        
        return horizontal_angle

    def bouy_angles(self, img):
        green_center, green_processed = self.__detect_green_buoy(img)
        red_center, red_processed = self.__detect_red_buoy(img)
        img_x, img_y = img.shape
        if green_center.any():
            green_x = green_center[0]
            green_pos_x = self.__sensor_position(green_x, img_x)
            green_horiz = self.__sensor_angles(green_pos_x)
        if red_center.any():
            red_x = red_center[0]
            red_pos_x = self.__sensor_position(red_x, img_x)
            red_horiz = self.__sensor_angles(red_pos_x)
        return (green_horiz, red_horiz)

