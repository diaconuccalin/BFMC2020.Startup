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
    pid = PID()
    
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
        super(LaneKeeping,self).__init__( inPs, outPs)

        self.serverIp   =  '192.168.0.199' # PC ip
        self.port       =  2244            # com port
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        self._init_socket()
        super(LaneKeeping,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        if self._blocker.is_set():
            return
        streamTh = Thread(name='StreamSending',target = self._send_thread, args= (self.inPs[0], ))
        streamTh.daemon = True
        self.threads.append(streamTh)

    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the socket. 
        """
        self.client_socket = socket.socket()
        self.connection = None

        # Trying repeatedly to connect the camera receiver.
        try:
            while self.connection is None and not self._blocker.is_set():
                try:
                    self.client_socket.connect((self.serverIp, self.port))
                    self.client_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                    self.connection = self.client_socket.makefile('wb') 
                except ConnectionRefusedError as error:
                    time.sleep(0.5)
                    pass
        except KeyboardInterrupt:
            self._blocker.set()
            pass

        
    # ===================================== SEND THREAD ==================================
    def _send_thread(self, inP):
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
                    if y2 == y1:
                        total = total + 10000
                    else:
                        total = total + (x2 - x1) / (y2 - y1)

            return total, img, lines


        def draw_lines(img, lines, color=[255, 0, 0], thickness=3):	
                # If there are no lines to draw, exit.	
                if lines is None:	
                    return	
                # Make a copy of the original image.	
                img = np.copy(img)	
                # Create a blank image that matches the original in size.	
                line_img = np.zeros(	
                    (	
                        img.shape[0],	
                        img.shape[1],	
                        3	
                    ),	
                    dtype=np.uint8,	
                )	

                # Loop over all lines and draw them on the blank image.	
                for line in lines:	
                    for x1, y1, x2, y2 in line:	
                        cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)	

                # Merge the image with the lines onto the original.	
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)	
                img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)	

                # Return the modified image.	
                return img	

        def prepareMask(img):
            kernel = np.ones((3, 3), np.uint8)
            img = cv2.erode(img, kernel, iterations=1)
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

            kernel = np.ones((5, 5), np.uint8)
            img = cv2.dilate(img, kernel, iterations = 1)

            return img
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        print('Start streaming')

        while True:
            try:
                stamps, img = inP.recv()

                val, img, lines = laneKeeping(img)
                img = draw_lines(img, lines)

                val = self.pid(val)
                print(val)

            except Exception as e:
                print(e)

            
