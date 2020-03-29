import socket
import struct
import time
import numpy as np
import datetime
from multiprocessing import Process
from threading import Thread

import cv2
import math

from src.utils.templates.workerprocess import WorkerProcess
from simple_pid import PID

class LaneKeeping(WorkerProcess):
    pid = PID(Ki = 0.01, Kd = 0.01)
    
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Process used for sending images over the network. UDP protocol is used. The
        image is compressed before it is send. 

        Used for visualizing your raspicam from PC.
        
        Parameters
        ----------
        inPs : list(Pipe) 
            List of input pipes, only the first pipe is used to transfer the captured frames. 
        outPs : list(Pipe) 
            List of output pipes (not used at the moment)
        """
        super(LaneKeeping,self).__init__(inPs, outPs)
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(LaneKeeping,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        if self._blocker.is_set():
            return
        streamTh = Thread(name='StreamSending',target = self._the_thread, args= (self.inPs[0], self.outPs[0], ))
        streamTh.daemon = True
        self.threads.append(streamTh)
        
    # ===================================== SEND THREAD ==================================
    def _the_thread(self, inP, outP):
        """Sending the frames received thought the input pipe to remote client by using a socket. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from other process. 
        """

        def laneKeeping(img):
            height = 480
            width = 640

            img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            img = img[(int(height/1.8)):height, 0:width]
            img = cv2.GaussianBlur(img, (7,7), 0)

            img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, -8)

            total = 0.0
            lines = cv2.HoughLinesP(img, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)

            for line in lines:
                for x1, y1, x2, y2 in line:
                    if y2 != y1:
                        total = total + (x2 - x1) / (y2 - y1)

            return total

        while True:
            try:
                stamps, img = inP.recv()

                val = laneKeeping(img)

                val /= 3

                val = self.pid(val)
                print(val)
                
                outP.send(val)

            except Exception as e:
                print("Lane keeping error:")
                print(e)

            
