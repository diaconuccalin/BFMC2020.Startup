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

class ImageGetter(WorkerProcess):
    
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Process used for capturing and sending the image stream to other processes that require it.
        
        Parameters
        ----------
        inPs : list(Pipe) 
            List of input pipes, only the first pipe is used to transfer the captured frames. 
        outPs : list(Pipe) 
            List of output pipes
        """
        super(ImageGetter,self).__init__( inPs, outPs)
        
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(ImageGetter,self).run()

    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        if self._blocker.is_set():
            return
        streamTh = Thread(name='StreamSending',target = self._send_thread, args= (self.inPs[0], ))
        streamTh.daemon = True
        self.threads.append(streamTh)

        
    # ===================================== SEND THREAD ==================================
    def _send_image(self, inP):
        """Sending the frames received thought the input pipe to remote client by using a socket. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from other process. 
        """
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
        
        while True:
            try:
                stamps, img = inP.recv()

                result, img = cv2.imencode('.jpg', img, encode_param)
                data   =  img.tobytes()
                size   =  len(data)

                self.connection.write(struct.pack("<L",size))
                self.connection.write(data)

            except Exception as e:
                print(e)
