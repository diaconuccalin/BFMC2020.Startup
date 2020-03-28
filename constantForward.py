from threading import Thread

from src.utils.templates.workerprocess import WorkerProcess

class ConstantForward(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, outPs):
        """Sets a constant speed to the vehicle
        
        Parameters
        ------------
        outPs : list(Pipe) 
            List of output pipes (order does not matter)
        """

        super(ConstantForward,self).__init__(outPs)

    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        sendTh = Thread(name='SendCommand',target = self._sendSpeed, args = (self.outPs, ))
        self.threads.append(sendTh)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(ConstantForward,self).run()

    def _sendSpeed(self, outPs, speed = 22.0):
        """Sends the requested speed to the microcontroller.
        
        Returns
        -------
        dict
            It contains the robot current control state, speed and angle. 
        """
        data = {}
        
        data['action'] = 'MCTL'
        data['speed'] = float(speed/100.0)
        
        try:
            for outP in outPs:
                outP.send(data)

        except Exception as e:
            print(e)
        