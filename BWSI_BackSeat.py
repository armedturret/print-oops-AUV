#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 16:18:55 2021

This is the simulated Sandshark front seat


@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import time
import threading
import datetime

from AUV_Controller import AUVController
from Logger import Logger

from pynmea2 import pynmea2
import BluefinMessages
from Sandshark_Interface import SandsharkClient

class BackSeat(): #Trial 4
    # we assign the mission parameters on init
    def __init__(self, host='localhost', port=8000, warp=1):
        
        # back seat acts as client
        self.__client = SandsharkClient(host=host, port=port)
        self.__current_time = time.time()
        self.__start_time = self.__current_time
        self.__warp = warp
        
        self.__logger = Logger()
        self.__autonomy = AUVController()
        #self.__logger = Logger()
    
    def run(self):
        try:
            # connect the client
            client = threading.Thread(target=self.__client.run, args=())
            client.start()

            msg = BluefinMessages.BPLOG('ALL', 'ON')
            self.send_message(msg)
            
            while True:
                now = time.time()
                delta_time = (now-self.__current_time) * self.__warp

                self.send_status()
                self.__current_time += delta_time
                
                msgs = self.get_mail()
                if len(msgs) > 0:
                    for msg in msgs:
                        self.process_message(msg)
                time.sleep(1/self.__warp)
                
                ### self.__autonomy.decide() probably goes here!
                command_str = self.__autonomy.decide()

                ### turn your output message into a BPRMB request! 
                if command_str != "":
                    for command in command_str.split(';'):
                        args = command.split(' ')
                        self.__current_time = time.time()
                        # This is the timestamp format from NMEA: hhmmss.ss
                        hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]
                        #check if a turn or thrust command
                        if args[0] == "turn" and len(args) == 2:
                            cmd = BluefinMessages.BPRMB(hhmmss, heading=args[1], horiz_mode=1)
                            self.send_message(cmd)
                        elif args[1] == "thruster" and len(args) == 2:
                            cmd = BluefinMessages.BPRMB(hhmmss, speed=args[1], speed_mode=0)
                            self.send_message(cmd)
                        
        except:
            self.__client.cleanup()
            client.join()
          
        
    def process_message(self, msg):
        # DEAL WITH INCOMING BFNVG MESSAGES AND USE THEM TO UPDATE THE
        # STATE IN THE CONTROLLER!
        self.__logger.log_event("RECIEVED", msg)
        
    def send_message(self, msg):
        self.__logger.log_event("SENT", msg)
        self.__client.send_message(msg)    
        
    def send_status(self):
        #print("sending status...")
        self.__current_time = time.time()
        hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]
        msg = BluefinMessages.BPSTS(hhmmss, 1, 'BWSI Autonomy OK')
        self.send_message(msg)
            
    def get_mail(self):
        msgs = self.__client.receive_mail()
        return msgs
    
            
def main():
    if len(sys.argv) > 1:
        host = sys.argv[1]
    else:
        host = "localhost"
        
    if len(sys.argv) > 2:
        port = int(sys.argv[2])
    else:
        port = 8042
    
    print(f"host = {host}, port = {port}")
    backseat = BackSeat(host=host, port=port)
    backseat.run()
    
            
if __name__ == '__main__':
    main()