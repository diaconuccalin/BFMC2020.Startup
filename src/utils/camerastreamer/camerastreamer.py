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
from multiprocessing import Process
from threading import Thread

import cv2
import math

from src.utils.templates.workerprocess import WorkerProcess

class CameraStreamer(WorkerProcess):
    count = 0.0
    summ = 0.0
    avg = 0.0

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

        self.serverIp   =  '192.168.0.220' # PC ip
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
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        print('Start streaming')

        while True:
            try:
                height = 480
                width = 640

                stamps, img = inP.recv()

                img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                
                img = img[(int(height/1.8)):height, 0:width]

                img = cv2.GaussianBlur(img, (7,7), 0)
                img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, -8)

                lines = cv2.HoughLinesP(img, rho=6, theta=np.pi/60, threshold=160, lines=np.array([]), minLineLength=40, maxLineGap=25)

                total = 0.0
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        if y2 == y1:
                            total = total + 10000
                        else:
                            total = total + (x2 - x1) / (y2 - y1)

                if(self.count < 30):
                    self.count = self.count + 1
                    self.summ = self.summ + total
                    print("test1")
                elif(self.count == 30):
                    self.avg = self.summ / 30
                    print("test2")
                else:
                    self.avg = (self.avg * 29 + total) / 30
                    print("test3")

                print(self.avg)

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

                img = draw_lines(img, lines)
                
                #kernel = np.ones((2,2), np.uint8)
                #img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
                #kernel = np.ones((7,7), np.uint8)
                #img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
                #img1, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)



                def cropRegion(img, vertices):
                    mask = np.zeros_like(img)
                    channelCount = 1
                    matchMaskColor = (255,) * channelCount
                    cv2.fillPoly(mask, vertices, matchMaskColor)
                    maskedImage = cv2.bitwise_and(img, mask)
                    return maskedImage
    
                roiVertices = [
                    (0, len(img)),
                    (len(img[0]) / 2, 0),
                    (len(img[0]), len(img))
                ]

                #img = cropRegion(img, np.array([roiVertices], np.int32),)

                 
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

            
