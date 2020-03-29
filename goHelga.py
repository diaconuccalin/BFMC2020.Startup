import sys
sys.path.append('.')

import time
import signal
from multiprocessing import Pipe, Process, Event 

from movementControl import MovementControl
from src.hardware.serialhandler.serialhandler import SerialHandler
from src.hardware.camera.cameraprocess import CameraProcess
from laneKeeping import LaneKeeping
from src.utils.camerastreamer.camerastreamer       import CameraStreamer

# =============================== CONFIG =================================================
enableConstantForward   =   True
enableLateralControl    =   True
enableStream            =   True
#================================ PROCESSES ==============================================
allProcesses = list()

lcR, lcS = Pipe(duplex = False)
camStR, camStS = Pipe(duplex = False)

if enableConstantForward:
    cfR, cfS = Pipe(duplex = False)

    cfProc = MovementControl([lcR], [cfS])
    allProcesses.append(cfProc)

    shProc = SerialHandler([cfR], [])
    allProcesses.append(shProc)

if enableLateralControl:
    lkR, lkS = Pipe(duplex = False)

    camProc = CameraProcess([],[lkS, camStS])
    allProcesses.append(camProc)

    lkProc = LaneKeeping([lkR], [lcS])
    allProcesses.append(lkProc)

    if enableStream:
        streamProc = CameraStreamer([camStR], [])
        allProcesses.append(streamProc)

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
