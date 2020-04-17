
# This controller runs the Image analysis pipeline without ROS Integration.
# Author: Walker Byrnes (walkerbyrnes@gmail.com)

from threading import Thread

from workerDetection import WorkerDetector
from locationFromDetection import LocationFromDetection
from findBin import WarehouseAisle
from databaseMessaging import IoTClient


class ControllerStandalone:
	""" This class controls the analysis pipeline for detecting workers. """

	def __init__(self, aisle=WarehouseAisle(1), basePath="~/catkin_ws/src/CapstoneBinTracking/", debug=False):

		print("Initializing Standalone Controller.")

		self.debug = debug
		self.basePath = basePath

		# Create class members for pipeline processes
		self.workerDetector = WorkerDetector(self.basePath)
		self.warehouse = aisle
		self.dbConn = IoTClient()

	def processImage(imageRGB, imageDepth):

		# Step 1: Detection
		detections = self.workerDetector(imageRGB)

		# Perform remaining steps for each detection in case of multiple workers in scene
		for det in detections:

			# Step 2: Localization
			detection3D = LocationFromDetection(imageRGB, imageDepth, det)

			if self.debug:
				print("Localization: ", detection3D)

			# Step 3: Bin Identification
			binStr = self.warehouse.findBin(detection3D)

			if self.debug:
				print("Bin Location: ", binStr)

			# Step 4: Database Connection
			self.dbConn.sendMessage(binStr)

			if self.debug:
				print("Database Message Sent")
				print("Processing pipeline complete.")