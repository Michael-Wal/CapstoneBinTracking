
# This controller runs the Image analysis pipeline without ROS Integration.
# Author: Walker Byrnes (walkerbyrnes@gmail.com)

from threading import Thread
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import cv2

from workerDetection import WorkerDetector
from locationFromDetection import LocationFromDetection
from findBin import WarehouseAisle
from trackInteractions import TrackWorkers
from databaseMessaging import IoTClient


DEFAULT_MODEL_PATH = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/resnet50_coco_best_v2.0.1.h5"
VIDEO_FPS = 8.0


class ControllerStandalone:
    """ This class controls the analysis pipeline for detecting workers. """

    def __init__(self, aisle=WarehouseAisle(1), modelPath=DEFAULT_MODEL_PATH, cameraOffset=None, debug=False, inspect=False):

        print("Initializing Standalone Controller.")

        self.debug = debug
        self.inspect = inspect
        self.cameraOffset = cameraOffset

        # Create class members for pipeline processes
        self.workerDetector = WorkerDetector(modelPath, debug=debug)
        self.warehouse = aisle
        self.tracker = TrackWorkers(debug=False)
        self.dbConn = IoTClient()

    def processImage(self, imageRGB, imageDepth):

        # Step 1: Detection
        detections, detImg = self.workerDetector.detectFromImage(np.asarray(imageRGB))

        # Perform remaining steps for each detection in case of multiple workers in scene
        for det in detections:

            # Step 2: Localization
            detection3D = LocationFromDetection(imageRGB, imageDepth, det, cameraLocationOffset=self.cameraOffset, debug=self.debug, inspect=self.inspect)

            if self.debug:
                print("Localization: ", detection3D)

            # Step 3: Bin Identification
            binStr = self.warehouse.findBin(detection3D)
            if binStr is not None:

                if self.debug:
                    print("Bin Location: ", binStr)

                # Step 4: Database Connection
                self.dbConn.sendMessage(binStr)

                if self.debug:
                    print("Database Message Sent")
                    print("Processing pipeline complete.")

    def processVideo(self, videoRGB, videoDepth, outputFile=False):

        print("Procesing videos:\nRGB: ", videoRGB, "\nDep: ", videoDepth)

        captureRGB = cv2.VideoCapture(videoRGB)
        captureDep = cv2.VideoCapture(videoDepth)

        if outputFile:
            outImages = []
            vidWrite = None
        while captureRGB.isOpened() and captureDep.isOpened():
            
            # Step 0: Extract frames from video and convert to PIL format
            retRGB, frameRGB = captureRGB.read()
            retDep, frameDep = captureDep.read()
            if not retRGB or not retDep:
                break

            # Convert frame to PIL format
            imageRGB = Image.fromarray(cv2.cvtColor(frameRGB, cv2.COLOR_BGR2RGB))
            imageDepth = Image.fromarray(frameDep)
            
            # Step 1: Detection
            detections, detImg = self.workerDetector.detectFromImage(np.asarray(imageRGB))

            # Perform remaining steps for each detection in case of multiple workers in scene
            bins = []
            for det in detections:

                # Step 2: Localization
                detection3D = LocationFromDetection(imageRGB, imageDepth, det, cameraLocationOffset=self.cameraOffset, debug=self.debug, inspect=self.inspect)

                if self.debug:
                    print("Localization: ", detection3D)

                # Step 3: Bin Identification
                binStr = self.warehouse.findBin(detection3D)
                if binStr is not None:
                    bins.append(binStr)

                    if self.debug:
                        print("Bin Location: ", binStr)

            # Step 3.5: Interaction tracking
            self.tracker.updateDetections(bins)

            for msg in self.tracker.sendDetections():

                # Step 4: Database Connection
                self.dbConn.sendMessage(binStr)

                if self.debug:
                    print("Database Message Sent")

            # Output video frames to file
            if outputFile:
                detImg = cv2.cvtColor(detImg, cv2.COLOR_BGR2RGB)
                if binStr is not None:

                    cv2.putText(detImg, binStr, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # cv2.imshow("image", detImg)
                # sleep(0.1)
                # cv2.destroyAllWindows()
    
                if not vidWrite:
                    print("Setting up video with resolution ", detImg.shape[:2])
                    vidWrite = cv2.VideoWriter("./detectionVid.avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), VIDEO_FPS, (detImg.shape[1], detImg.shape[0]))
                
                print("Writing frame to video")
                vidWrite.write(detImg)

        if vidWrite:
            vidWrite.release()


        print("Video processing complete")


# Test full analysis pipeline
if __name__ == "__main__":

    # Estimated camera matrix for test image
    camOffset = np.asarray([[1, 0, 0, -1.9], [0, 0, 1, 0.0], [0, 1, 0, 1.5]])
    print("Camera Matrix:\n", camOffset)

    TEST_WITH_VIDEO = True

    PathRGB = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testcolorvid.mp4"
    PathDep = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testdepthvid.mp4"

    if TEST_WITH_VIDEO:

        cs = ControllerStandalone(cameraOffset=camOffset, debug=True, inspect=False)

        # Process videos
        cs.processVideo(PathRGB, PathDep, outputFile=True)

    else:

        cs = ControllerStandalone(cameraOffset=camOffset, debug=True, inspect=True)

        # Import test RGB image
        imgRGB = Image.open(PathRGB)

        # Import test Depth image
        imgDepth = Image.open(PathDep)

        print("\n\nSetup complete, calling analysis pipeline...\n\n")
        cs.processImage(imgRGB, imgDepth)
