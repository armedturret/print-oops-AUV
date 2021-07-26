#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!
### MDM: added Rasperry Pi V2 camera 

import sys
import pathlib
import datetime

import time 
import numpy as np
import matplotlib.pyplot as plt
#import picamera 
#import picamera.array

import cv2

# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera


class ImageProcessor():
    def __init__(self, camera='SIM', log_dir='./'):
        self.__camera_type = camera.upper()

        if self.__camera_type == 'SIM':
            self.__camera = BWSI_Camera(max_angle=31.1, visibility=50)
            self.__simField = None
            
        else:
            #self.__camera = picamera.PiCamera()
            self.__camera.resolution = (640, 480)
            self.__camera.framerate = 24
            time.sleep(2) # camera warmup time
            self.__image = np.empty((480*640*3,), dtype=np.uint8)

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

        object_detection_surface = cv2.boxFilter(img_thresh_HSV.astype(int), -1, (50,50), normalize=False)

        centers = np.argwhere(object_detection_surface > 50)
        center = np.mean(centers, axis=0) if centers.shape[0] > 0 else np.array([])
        
        print("Green Centre " + str(center))
        return center

    def __detect_red_buoy(self, img):
        img = cv2.resize(img, (640, 480))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        imhsv = cv2.boxFilter(img, -1, (10,10))
        img_thresh_hue = np.logical_and( imhsv[:,:,0] > 100, imhsv[:,:,0] < 150)
        img_thresh_sat = np.logical_and( imhsv[:,:,1] > 60, imhsv[:,:,1] < 120)
        img_thresh_val = np.logical_and( imhsv[:,:,2] > 200, imhsv[:,:,2] < 240)
        img_thresh_HSV = np.logical_and(img_thresh_hue, img_thresh_sat, img_thresh_val)
        object_detection_surface = cv2.boxFilter(img_thresh_HSV.astype(int), -1, (50,50), normalize=False)

        centers = np.argwhere(object_detection_surface > 50)
        center = np.mean(centers, axis=0) if centers.shape[0] > 0 else np.array([])
        
        print("Red Center " + str(center))
        return center

    def __sensor_position(pix_x, res_x): 
        sensor_pos_x = (pix_x - (res_x / 2.0)) / res_x * 3.68

        return sensor_pos_x

    def __sensor_angles(pos_x):
        focal_length = 3.04

        horizontal_angle = np.arctan2(pos_x,focal_length)
        horizontal_angle = np.degrees(horizontal_angle)
        
        return horizontal_angle

    def __buoy_angles(self, img):
        green_center = self.__detect_green_buoy(img)
        red_center = self.__detect_red_buoy(img)
        img_x = img.shape[1]
        green_horiz = list()
        red_horiz = list()
        if green_center.any():
            green_x = green_center[0]
            green_pos_x = self.__sensor_position(green_x, img_x)
            #green_horiz = list(__sensor_angles(green_pos_x))
            g_sa = self.__sensor_angles(green_pos_x)
            green_horiz.append(g_sa)
        if red_center.any():
            red_x = red_center[0]
            red_pos_x = self.__sensor_position(red_x, img_x)
            r_sa = self.__sensor_angles(red_pos_x)
            red_horiz.append(r_sa)
        return (green_horiz, red_horiz) 
    
    # ------------------------------------------------------------------------ #
    # Run an iteration of the image processor. 
    # The sim version needs the auv state to generate simulated imagery
    # the PICAM does not need any auv_state input
    # ------------------------------------------------------------------------ #
    def run(self, auv_state=None):
        red = list()
        green = list()
        if auv_state['heading'] is not None:
            if (self.__camera_type == 'SIM'):
                # if it's the first time through, configure the buoy field
                if self.__simField is None:
                    self.__simField = BuoyField(auv_state['datum'])
                    config = {'nGates': 5,
                              'gate_spacing': 5,
                              'gate_width': 2,
                              'style': 'pool_1',
                              'max_offset': 5,
                              'heading': 0}
                    
                    self.__simField.configure(config)
                 
                image = self.__camera.get_frame(auv_state['position'], auv_state['heading'], self.__simField).astype('float32')

            elif self.__camera_type == 'PICAM':
                try:
                    self.__camera.capture(self.__image, 'bgr')
                except:
                    # restart the camera
                    #self.__camera = picamera.PiCamera()
                    self.__camera.resolution = (640, 480)
                    self.__camera.framerate = 24
                    time.sleep(2) # camera warmup time
                    
                image = self.__image.reshape((480, 640, 3))
        
            else:
                print(f"Unknown camera type: {self.__camera_type}")
                sys.exit(-10)
        
            # log the image
            fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
            cv2.imwrite(str(fn), image)
        
            green, red = self.__buoy_angles(image)

            print("Green Angle: " + str(green))
            print("Red Angle: " + str(red))
        
        return red, green
