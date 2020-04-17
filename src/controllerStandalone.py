
# This controller runs the Image analysis pipeline without ROS Integration.
# Author: Walker Byrnes (walkerbyrnes@gmail.com)

from threading import Thread
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from cv2 import VideoCapture, cvtColor, COLOR_BGR2RGB

from workerDetection import WorkerDetector
from locationFromDetection import LocationFromDetection
from findBin import WarehouseAisle
from trackInteractions import TrackWorkers
from databaseMessaging import IoTClient


DEFAULT_MODEL_PATH = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/resnet50_coco_best_v2.0.1.h5"


class ControllerStandalone:
    """ This class controls the analysis pipeline for detecting workers. """

    def __init__(self, aisle=WarehouseAisle(1), modelPath=DEFAULT_MODEL_PATH, cameraOffset=None, debug=False, inspect=False):

        print("Initializing Standalone Controller.")

        self.debug = debug
        self.inspect = inspect
        self.cameraOffset = cameraOffset

        # Create class members for pipeline processes
        self.workerDetector = WorkerDetector(modelPath, debug=debug, inspect=inspect)
        self.warehouse = aisle
        self.tracker = TrackWorkers(debug=False)
        self.dbConn = IoTClient()

    def processImage(self, imageRGB, imageDepth):

        # Step 1: Detection
        detections = self.workerDetector.detectFromImage(np.asarray(imageRGB))

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

    def processVideo(self, videoRGB, videoDepth):

        print("Procesing videos:\nRGB: ", videoRGB, "\nDep: ", videoDepth)

        captureRGB = VideoCapture(videoRGB)
        captureDep = VideoCapture(videoDepth)

        while captureRGB.isOpened() and captureDep.isOpened():
            
            # Step 0: Extract frames from video and convert to PIL format
            retRGB, frameRGB = captureRGB.read()
            retDep, frameDep = captureDep.read()
            if not retRGB or not retDep:
                break

            # Convert frame to PIL format
            imageRGB = Image.fromarray(cvtColor(frameRGB, COLOR_BGR2RGB))
            imageDepth = Image.fromarray(frameDep)
            
            # Step 1: Detection
            detections = self.workerDetector.detectFromImage(np.asarray(imageRGB))

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

        print("Video processing complete")


# Test full analysis pipeline
if __name__ == "__main__":

    # Estimated camera matrix for test image
    camOffset = np.asarray([[-1, 0, 0, -1.5], [0, 0, 1, 0.0], [0, 1, 0, 1.0]])
    print("Camera Matrix:\n", camOffset)

    TEST_WITH_VIDEO = True

    if TEST_WITH_VIDEO:

        cs = ControllerStandalone(cameraOffset=camOffset, debug=True, inspect=False)

        # Import videos
        videoPathRGB = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testcolorvid.mp4"
        videoPathDep = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testdepthvid.mp4"

        # Process videos
        cs.processVideo(videoPathRGB, videoPathDep)

    else:

        cs = ControllerStandalone(cameraOffset=camOffset, debug=True, inspect=True)

        # Import test RGB image
        imagePathRGB = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testcolor.jpg"  # TEST IMAGE PATH
        imgRGB = Image.open(imagePathRGB)
        # plt.imshow(imgRGB)
        # plt.show()

        # Import test Depth image
        imagePathDepth = "/home/walker/catkin_ws/src/CapstoneBinTracking/data/testdepth.jpg"  # TEST IMAGE PATH
        imgDepth = Image.open(imagePathDepth)
        # plt.imshow(imgDepth)
        # plt.show()

        print("\n\nSetup complete, calling analysis pipeline...\n\n")
        cs.processImage(imgRGB, imgDepth)
