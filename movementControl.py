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
        self.angle = 0.0
        self.speed = 16.0

        super(MovementControl,self).__init__(inPs, outPs)

    def _init_threads(self):
        """Initialize the read thread to transmite the received messages to other processes. 
        """

        startTh = Thread(name='InitialStart', target = self._singleUpdate, args=(self.outPs, ))
        self.threads.append(startTh)

        sendTh = Thread(name='SteeringListen',target = self._listen_for_steering, args = (self.inPs[0], self.outPs, ))
        self.threads.append(sendTh)
        

    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        super(MovementControl,self).run()

    def stop(self):
        self.speed = 0.0
        self.angle = 0.0
        self._singleUpdate(self.outPs)
        super(MovementControl, self).stop()

    def _listen_for_steering(self, inP, outPs):
        while True:
            try:
                value = inP.recv()
                self.angle = float(value)
                self._singleUpdate(outPs)
            except Exception as e:
                print("Listening error:")
                print(e)

    def _singleUpdate(self, outPs):
        data = {}
        if(self.speed != 0):
            data['action'] = 'MCTL'
            data['speed'] = float(self.speed/100.0)
        else:
            data['action'] = 'BRAK'
        data['steerAngle'] = self.angle
        
        try:
            for outP in outPs:
                outP.send(data)

        except Exception as e:
            print(e)
            
        