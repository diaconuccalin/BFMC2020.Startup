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
import io
import cv2
import numpy as np
        
import time
import datetime


import threading

from threading import Thread
from multiprocessing import Process
from src.utils.templates.threadwithstop import ThreadWithStop

#================================ CAMERA PROCESS =========================================
class CameraPublisher(ThreadWithStop):
    
    #================================ CAMERA =============================================
    def __init__(self, outPs):
        """The purpose of this thread is to send the camera images. It is able to record
        videos and save them locally.
        
        Parameters
        ----------
        outPs : list(Pipes)
            the list of pipes were the images will be sent
        """
        super(CameraPublisher,self).__init__()
        self.daemon = True


        # streaming options
        self._stream      =   io.BytesIO()
        
        #output 
        self.outPs         =   outPs

    #================================ INIT CAMERA ========================================
    def _init_camera(self):
        """Init the PiCamera and its parameters
        """
        
        # this how the firmware works.
        # the camera has to be imported here
        from picamera import PiCamera

        # camera
        self.camera = PiCamera()

        # camera settings
        self.camera.resolution      =   (640,480)
        self.camera.framerate       =   20

        self.camera.brightness      =   50  # default
        self.camera.shutter_speed   =   0   # auto
        self.camera.contrast        =   0   # default
        self.camera.iso             =   0   # auto
        self.camera.awb_mode        =   'off'
        self.camera.awb_gains       =   (1.0, 1.0)
        

        self.imgSize                =   (640, 480)    # the actual image size
        self.recordMode             =   False


        # WB calibration

        # Obtain sample image
        time.sleep(5)

        img = np.zeros((480, 640, 3), np.uint8)
        self.camera.capture(img, format = 'rgb')

        # Obtain ROI
        height = img.shape[0]
        width = img.shape[1]

        img = img[(int(0.7*height)):(int(0.9*height)), (int(0.3*width)):(int(0.7*width))]

        height = img.shape[0]
        width = img.shape[1]

        # Compute rgb levels in ROI
        reds = 0.0
        blues = 0.0
        greens = 0.0

        r, g, b = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        for i in range(height):
            for j in range(width):
                reds += r[i, j]
                blues += b[i, j]
                greens += g[i, j]

        print(reds)
        print(greens)
        print(blues)
        
        # Compute and apply gains
        reds = greens / reds
        blues = greens / blues

        self.camera.awb_gains = (reds, blues)

    # ===================================== GET STAMP ====================================
    def _get_timestamp(self):
        stamp = time.gmtime()
        res = str(stamp[0])
        for data in stamp[1:6]:
            res += '_' + str(data)  

        return res
    #================================ RUN ================================================
    def run(self):
        """Start sending data through pipe. 
        """
        self._init_camera()
        
        # record mode
        if self.recordMode:
            self.camera.start_recording('picam'+ self._get_timestamp()+'.h264',format='h264')

        self.camera.capture_sequence(
                                    self._streams(), 
                                    use_video_port  =   True, 
                                    format          =   'rgb',
                                    resize          =   self.imgSize)
        # record mode
        if self.recordMode:
            self.camera.stop_recording()
     
    #================================ STREAMS ============================================
    def _streams(self):
        """Stream function that actually published the frames into the pipes. Certain 
        processing(reshape) is done to the image format. 
        """
        i = 0

        while self._running:
            
            yield self._stream
            self._stream.seek(0)
            data = self._stream.read()

            # read and reshape from bytes to np.array
            data  = np.frombuffer(data, dtype=np.uint8)
            data  = np.reshape(data, (480, 640, 3))
            stamp = time.time()

            # output image and time stamp
            # Note: The sending process can be blocked, when doesn't exist any consumer process and it reaches the limit size.
            for outP in self.outPs:
                outP.send([[stamp], data])

            
            self._stream.seek(0)
            self._stream.truncate()


