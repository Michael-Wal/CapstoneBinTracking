from picamera import PiCamera
from datetime import datetime
from time import sleep
import os

CLIP_TIME_MIN = 5.0/60.0
NUM_CLIPS = 1 # 0 for infinite

class SimpleCaptureServer:

    def __init__(self):
        # Create camera
        self.cam = PiCamera()
        try:
            self.cam.start_preview(fullscreen=False, window=(40, 40, 1280, 720))

            if NUM_CLIPS == 0:
                while true:
                    self.recordClip(CLIP_TIME_MIN)
            else:
                clipsLeft = NUM_CLIPS
                while clipsLeft > 0:
                    self.recordClip(CLIP_TIME_MIN)
                    clipsLeft = clipsLeft - 1 
        finally:
            # Close camera module on termination
            self.onDestroy()

    def recordClip(self, clipTime, fPath=None):
        """ Record a single clip of time clipTime. 
        
        Arguments:
            clipTime -- Duration of clip in MINUTES
            fPath -- Optional parameter, save location for clip
        """
        
        # Fail if camera has not been initiated
        if not self.cam:
            return None

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
