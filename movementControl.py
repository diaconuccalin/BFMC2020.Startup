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

        self.speed = 19.0
        self.angle = 0.0
        self._update()

        super(MovementControl,self).__init__(inPs, outPs)

    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """
        sendTh = Thread(name='SteeringListen',target = self._listen_for_steering, args = (self.inPs[0], ))
        self.threads.append(sendTh)

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(MovementControl,self).run()

    def stop(self):
        self.speed = 0.0
        self._update()
        super(MovementControl, self).stop()

    def _listen_for_steering(self, inP):
        while True:
            try:
                value = inP
                print(value)
            except Exception as e:
                print("Listening error:")
                print(e)

    def _update(self):
        """Sends the requested speed to the microcontroller.
        
        Returns
        -------
        dict
            It contains the robot current control state, speed and angle. 
        """

        data = {}
        
        if(self.speed != 0):
            data['action'] = 'MCTL'
            data['speed'] = float(self.speed/100.0)
        else:
            data['action'] = 'BRAK'
        data['steerAngle'] = self.angle
        
        try:
            for outP in self.outPs:
                outP.send(data)

        except Exception as e:
            print(e)
        