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
        """
        def cropRegion(img, vertices):
            mask = np.zeros_like(img)
            channelCount = 1
            matchMaskColor = (255,) * channelCount
            cv2.fillPoly(mask, vertices, matchMaskColor)
            maskedImage = cv2.bitwise_and(img, mask)
            return maskedImage

        def processForLane(img):
            width = 700
            height = 400

            img = cv2.resize(img, (width, height), interpolation = cv2.INTER_AREA)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img = img[(int(height/1.8)):(height - 30), (int(width*0.2)):(width - (int(width*0.15)))]
            img = cv2.GaussianBlur(img, (5,1), 0)
            img1, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

            roiVertices = [
                (0, len(img)),
                (len(img[0]) / 2, 0),
                (len(img[0]), len(img))
            ]

            img = cropRegion(img, np.array([roiVertices], np.int32),)

            return img
        """    
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        print('Start streaming')

        while True:
            try:
                stamps, img = inP.recv()

                width = 700
                height = 400
                result, img = cv2.imencode('.jpg', img, encode_param)

                print(img.shape)
                
                #img = cv2.resize(img, (width, height), interpolation = cv2.INTER_AREA)
                #img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
                #img = img[(int(height/1.8)):(height - 30), (int(width*0.2)):(width - (int(width*0.15)))]
                #img = cv2.GaussianBlur(img, (5,1), 0)
                #img1, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

                #roiVertices = [
                #    (0, len(img)),
                #    (len(img[0]) / 2, 0),
                #    (len(img[0]), len(img))
                #]

                #img = cropRegion(img, np.array([roiVertices], np.int32),)
                 
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

            
