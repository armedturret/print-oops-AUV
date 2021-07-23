#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!

import sys
import pathlib
import datetime

import cv2

# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera


class ImageProcessor():
    def __init__(self, camera='PICAM', log_dir='./'):
        self.__camera_type = camera.upper()

        if self.__camera_type == 'SIM':
            self.__camera = BWSI_Camera(max_angle=31.1, visibility=30)
            self.__simField = None
            
        else:
            pass

        # create my save directory
        self.__image_dir = pathlib.Path(log_dir, 'frames')
        self.__image_dir.mkdir(parents=True, exist_ok=True)


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

    def __bouy_angles(self, img):
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
    
    # ------------------------------------------------------------------------ #
    # Run an iteration of the image processor. 
    # The sim version needs the auv state ot generate simulated imagery
    # the PICAM does not need any auv_state input
    # ------------------------------------------------------------------------ #
    def run(self, auv_state=None):
        red = None
        green = None
        if auv_state['heading'] is not None:
            if (self.__camera_type == 'SIM'):
                if self.__simField is None:
                    self.__simField = BuoyField(auv_state['datum'])
                    config = {'nGates': 50,
                              'gate_spacing': 20,
                              'gate_width': 2,
                              'style': 'linear',
                              'max_offset': 5,
                              'heading': 120}
                    
                    self.__simField.configure(config)
                 
                image = self.__camera.get_frame(auv_state['position'], auv_state['heading'], self.__simField)

            elif self.__camera_type == 'PICAM':
                pass
        
            else:
                print(f"Unknown camera type: {self.__camera_type}")
                sys.exit(-10)
        
            # log the image
            fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
            cv2.imwrite(str(fn), image)
        
            green, red = self.__bouy_angles(image)
        
        return red, green
