import numpy as np
import datetime

class Logger():
    def __init__(self, print_events=False):
        timestamp = str(np.datetime64(datetime.datetime.now())).replace(':','')
        self.__file = open(f"logs/log-{timestamp}.txt", "a")
        self.__npfile = open(f"logs/data-{timestamp}.out", "a")
        self.__start = np.datetime64(datetime.datetime.now()) 
        self.__print_events = print_events

    #logs an event in format timestamp [source]: message
    def log_event(self, source, message):
        timestamp = str(np.datetime64(datetime.datetime.now()))
        formatted = f"[{timestamp}] [{source}]: {message}\n"
        if self.__print_events:
            print(formatted)
        self.__file.write(formatted)

    def log_auv_location(self, x, y, heading):
        float_time = (np.datetime64(datetime.datetime.now()) - self.__start) / np.timedelta64(1, 'ms')
        arr = np.array([[float_time, x, y, heading]])
        np.savetxt(self.__npfile, arr, delimiter=',')

