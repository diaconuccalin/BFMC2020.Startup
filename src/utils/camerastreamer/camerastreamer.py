# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
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

class CameraStreamer(WorkerProcess):
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
        super(CameraStreamer,self).__init__( inPs, outPs)

        self.serverIp   =  '192.168.0.199' # PC ip
        self.port       =  2244            # com port
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        self._init_socket()
        super(CameraStreamer,self).run()

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

        def signDetection(img):
            original = img.copy()

            # Crop top right corner
            height = img.shape[0]
            width = img.shape[1]
            img = img[0:(int)(height/2), (int)(width/2):width]

            height = img.shape[0]
            width = img.shape[1]

            # Remove noise
            img = cv2.GaussianBlur(img, (5, 5), 0)

            # Obtain hue
            h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))

            # Create masks for red, blue, yellow
            ret, r1 = cv2.threshold(h, 112, 255, cv2.THRESH_BINARY)
            ret, y1 = cv2.threshold(h, 107, 255, cv2.THRESH_BINARY)

            h = cv2.bitwise_not(h)

            ret, r2 = cv2.threshold(h, 131, 255, cv2.THRESH_BINARY)
            ret, b = cv2.threshold(h, 250, 255, cv2.THRESH_BINARY)
            ret, y2 = cv2.threshold(h, 145, 255, cv2.THRESH_BINARY)
            
            r = cv2.bitwise_and(r1, r2)
            y = cv2.bitwise_and(y1, y2)

            # Morphologies on masks
            r = prepareMask(r)
            b = prepareMask(b)
            y = prepareMask(y)

            # To display
            h = cv2.cvtColor(h, cv2.COLOR_GRAY2BGR)
            r = cv2.cvtColor(r, cv2.COLOR_GRAY2BGR)
            b = cv2.cvtColor(b, cv2.COLOR_GRAY2BGR)
            y = cv2.cvtColor(y, cv2.COLOR_GRAY2BGR)

            topRow = np.concatenate((img, r), axis = 1)
            bottomRow = np.concatenate((b, y), axis = 1)
            img = np.concatenate((topRow, bottomRow), axis = 0)

            return img
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        print('Start streaming')

        while True:
            try:
                stamps, img = inP.recv()

                val, img, lines = laneKeeping(img)
                img = draw_lines(img, lines)

                #val = self.pid(val)
                #print(val)

                #f = open("log.txt", "a")

                #now = datetime.datetime.now()
                #f.write(str(now.strftime("\n %Y-%m-%d %H:%M:%S  -  ")))

                #val = str(val)
                #f.write(val)

                #f.close()

                #img = signDetection(img)

                #height = img.shape[0]
                #width = img.shape[1]

                #img = img[(int(0.7*height)):(int(0.9*height)), (int(0.3*width)):(int(0.7*width))]

                result, img = cv2.imencode('.jpg', img, encode_param)
                data   =  img.tobytes()
                size   =  len(data)

                self.connection.write(struct.pack("<L",size))
                self.connection.write(data)

            except Exception as e:
                print("CameraStreamer failed to stream images:",e,"\n")
                # Reinitialize the socket for reconnecting to client.  
                self.connection = None
                self._init_socket()
                pass

            
