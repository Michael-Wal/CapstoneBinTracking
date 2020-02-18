from picamera import PiCamera
from datetime import datetime
from time import sleep
from threading import Thread
from queue import Queue
import os
import sys

CLIP_TIME_MIN = 5/60.0
NUM_CLIPS = 0 # 0 for infinite

class SimpleCaptureServer:

    def __init__(self):
        # Create camera and input buffer
        self.cam = PiCamera()
        print("Camera Initialized.")

        # Start user input thread
        self.inputQueue = Queue()
        self.inputKillCmd = True
        self.inputThread = Thread(target=self.getUserInput, args=(self.inputQueue, self.inputKillCmd))
        self.inputThread.start()
        print("User Input Thread Created.")
        print("Press any key to exit program.")

        # Start camera loop
        try:
            self.cam.start_preview(fullscreen=False, window=(10, 10, 1280, 720))

            recordSuccess = True
            clipNum = 1
            while ((clipNum <= NUM_CLIPS or NUM_CLIPS <= 0) and recordSuccess \
                    and self.inputThread.isAlive() and self.inputQueue.empty()):

                # Create recording clip in a new thread
                print("Recording Clip number ", clipNum)
                recThread = Thread(target=lambda: self.recordClip(CLIP_TIME_MIN))
                recThread.start()
                
                # Wait for user input or thread to complete
                while recThread.isAlive() and self.inputThread.isAlive() and self.inputQueue.empty(): 
                    sleep(1)
                
                clipNum = clipNum + 1 
        
        finally:
            # Close camera module on termination
            print("Closing Camera.")
            self.onDestroy()
            print("If process continues to run, press any key to exit.")

    def getUserInput(self, inputQueue, killCmd):
        """ Handle user input events in order to kill program. """
        while inputQueue.empty():
            inputQueue.put(sys.stdin.read(1))

    def recordClip(self, clipTime, fPath=None):
        """ Record a single clip of time clipTime. 
        
        Arguments:
            clipTime -- Duration of clip in MINUTES
            fPath -- Optional parameter, save location for clip
        """
        
        # Fail if camera has not been initiated
        if not self.cam:
            return False

        # If clip path not given, use default path
        if not fPath:
            fPath = "/home/pi/Videos/pyCamera/"

        # Record clip and save video file
        fullPath = fPath + datetime.now().strftime("%Y-%m-%d_T%H_%M_%S") + ".h264"
        self.cam.start_recording(fullPath)
        sleep(clipTime * 60.0)
        self.cam.stop_recording()

        self.checkOverflow(fPath)
        print("Saved clip to ", fullPath)
        return True

    def checkOverflow(selfi, fPath):
        """ Check size of directory and remove old files if necessary."""

        # Print out listdir
        savedFiles = os.listdir(fPath)
        savedFiles.sort()
        print(savedFiles[0])
        # Return true if a file was deleted
        return False

    # Close preview when application is ended
    def onDestroy(self):
        if self.cam is not None:
            self.cam.stop_preview()
            self.cam.close()
            

# Main function, create video server
if __name__ == "__main__":

    SimpleCaptureServer()
