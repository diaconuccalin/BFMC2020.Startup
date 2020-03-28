from threading import Thread

from src.utils.templates.workerprocess import WorkerProcess

class MovementControl(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Controls the speed and steering of the vehicle
        
        Parameters
        ------------
        inPs  : list(Pipe)
            List of input pipes (not used)
        outPs : list(Pipe) 
            List of output pipes (order does not matter)
        """

        super(MovementControl,self).__init__(inPs, outPs)

    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        #sendTh = Thread(name='SendCommand',target = self._sendSpeed, args = ())
        #self.threads.append(sendTh)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(MovementControl,self).run()

    def stop(self):
        self._sendSpeed(speed = 0.0)
        super(MovementControl, self).stop()

    def _sendSpeed(self, speed = 19.0):
        """Sends the requested speed to the microcontroller.
        
        Returns
        -------
        dict
            It contains the robot current control state, speed and angle. 
        """
        data = {}
        
        if(speed != 0):
            data['action'] = 'MCTL'
            data['speed'] = float(speed/100.0)
        else:
            data['action'] = 'BRAK'
        data['steerAngle'] = 0.0
        
        try:
            for outP in self.outPs:
                outP.send(data)

        except Exception as e:
            print(e)
        