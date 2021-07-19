#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 16 23:04:06 2021

@author: BWSI AUV Challenge Instructional Staff
"""

import time, sys, os

import socket
import select
import threading
import queue

import numpy as np

from pynmea2 import pynmea2
import BluefinMessages

## The main message handler
class SandsharkServer():
    def __init__(self,
                 host="",
                 port=8000,
                 PACKET_SIZE=1024): #PACKET?

        self.__host = host
        self.__port = port
        self.__PACKET_SIZE = PACKET_SIZE
        
        # bind to port
        self.__sockt = socket.socket(socket.AF_INET,
                                     socket.SOCK_STREAM) #socket is a wire
        self.__sockt.bind((host, port))
        self.__sockt.listen(5)
        self.__outgoing = queue.Queue(maxsize=10) #for outgoing msgs, max = 10
        self.__incoming = queue.Queue(maxsize=50) #for incoming msgs, max = 50
        
    def run(self):
        try:
            inputs = [ self.__sockt ]
            outputs = [ ]
            
            while inputs:
                readable, writeable, exceptional = select.select(inputs, outputs, inputs)
                
                for s in readable:
                    if s is self.__sockt:
                        connection, client_address = s.accept()
                        #print(f"New connection from {client_address}")
                        connection.setblocking(0)
                        inputs.append(connection)
                    else:
                        data = s.recv(self.__PACKET_SIZE)
                        if data:
                            self.__incoming.put(data)
                            #print(f"Received: {str(data, 'UTF-8')}")
                            if s not in outputs:
                                # use this open connection to send data
                                outputs.append(s) 
                        else: # closed
                            if s in outputs:
                                outputs.remove(s)
                            inputs.remove(s)
                            s.close()
                for s in writeable:
                    while not self.__outgoing.empty():
                        try:
                            next_msg = self.__outgoing.get()
                        except:
                            outputs.remove(s)
                        else:
                            print(f"To Backseat: {str(next_msg, 'utf-8')}\n")
                            s.send(next_msg)
                            self.__outgoing.task_done()
                            
                for s in exceptional:
                    inputs.remove(s)
                    if s in outputs:
                        outputs.remove(s)
                    s.close()                
        except:
            self.cleanup()
            
    # send message to the payload - try without a worker thread for now
    def send_command(self, cmd):
        if self.__outgoing.full():
            _ = self.__outgoing.get() # dump message
            self.__outgoing.task_done()
        self.__outgoing.put(bytes(cmd, 'utf-8'))
                    
    def cleanup(self):
        self.__sockt.close()
            
    def __listener_thread(self, client): #thread for listening
        done = False
        msg_list = list()
        while not done:
            data = client.recv(self.__PACKET_SIZE)
        
            if data:
                msg_list.append(str(data, 'utf-8'))
            else:
                done = True
        
        client.close()
        
        self.__handle_data(msg_list)
        
    # interpret the received strings
    def receive_mail(self):
        your_mail = list()
        while not self.__incoming.empty():
            msg = self.__incoming.get()
            your_mail.append(msg)
            self.__incoming.task_done()
            
        return your_mail

    
# the backseat acts as client
class SandsharkClient():
    def __init__(self,
                 host="localhost",
                 port=8000,
                 PACKET_SIZE=1024):

        self.__host = host
        self.__port = port
        self.__PACKET_SIZE = PACKET_SIZE
        
        # bind to port
        self.__sockt = socket.socket(socket.AF_INET,
                                     socket.SOCK_STREAM)
        connected = False
        while not connected:
            try:
                self.__sockt.connect((host, port))
                connected = True
            except:
                print("waiting to connect to front seat...")
                time.sleep(1)
        self.__outgoing = queue.Queue(maxsize=10)
        self.__incoming = queue.Queue(maxsize=50)
        
    def run(self):
        try:
            while True:
                if not self.__outgoing.empty():
                    next_msg = self.__outgoing.get()
                    try:
                        self.__sockt.connect((self.__host, self.__port))
                    except:
                        pass
                    self.__sockt.send(next_msg)
                    self.__outgoing.task_done()
    
                    time.sleep(0.01)
                    
                    # now check for messages in return
                    data = self.__sockt.recv(self.__PACKET_SIZE)
                    if data:
                        #print(f"Putting {str(data, 'utf-8')}")
                        self.__incoming.put(data)
                        
                time.sleep(0.01)
        except:
            self.cleanup()
            
    # send command to the vehicle - try without a worker thread for now
    def send_message(self, cmd):
        if self.__outgoing.full():
            _ = self.__outgoing.get() # dump message
            self.__outgoing.task_done()
        self.__outgoing.put(bytes(cmd, 'utf-8'))
        
    # pick up whatever messages have been accumulated since last request
    def receive_mail(self):        
        your_mail = list()
        while not self.__incoming.empty():
            msg = self.__incoming.get()
            your_mail.append(msg)
            self.__incoming.task_done()
            
        return your_mail

                    
    def cleanup(self):
        self.__sockt.close()
            
    def __listener_thread(self, client):
        done = False
        msg_list = list()
        while not done:
            data = client.recv(self.__PACKET_SIZE)
        
            if data:
                msg_list.append(str(data, 'utf-8'))
            else:
                done = True
        
        client.close()
        
        self.__handle_data(msg_list)
        
    # interpret the received strings
    def __handle_data(self, data):
        print(f"Received {data}")
        #nmea.parse(data)        
        
# for unit test
def main():
    pass                        

if __name__ == "__main__":
    main()
    #hello