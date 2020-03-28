import sys
sys.path.append('.')

import time
import signal
from multiprocessing import Pipe, Process, Event 

# utility imports
from movementControl import MovementControl
from src.hardware.serialhandler.serialhandler import SerialHandler


# =============================== CONFIG =================================================
enableConstantForward   =   True
enableLateralControl    =   False

#================================ PROCESSES ==============================================
allProcesses = list()

if enableConstantForward:
    cfR, cfS = Pipe(duplex = False)

    cfProc = MovementControl([], [cfS])
    allProcesses.append(cfProc)

    shProc = SerialHandler([cfR], [])
    allProcesses.append(shProc)

# Starting the processes
print("Starting the processes!",allProcesses)
for proc in allProcesses:
    proc.daemon = True
    proc.start()

# Waiting for keyboard interruption
blocker = Event()

try:
    blocker.wait()
except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")
    for proc in allProcesses:
        if hasattr(proc,'stop') and callable(getattr(proc,'stop')):
            print("Process with stop",proc)
            proc.stop()
            proc.join()
        else:
            print("Process witouth stop",proc)
            proc.terminate()
            proc.join()
