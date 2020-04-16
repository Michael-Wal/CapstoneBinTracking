
# This controller runs the Image analysis pipeline without ROS Integration.
# Author: Walker Byrnes (walkerbyrnes@gmail.com)

from threading import Thread

from locationFromDetection import LocationFromDetection
from findBin_direct import findBin
from databaseMessaging import IoTClient


class ControllerStandalone:
	""" This class controls the analysis pipeline for detecting workers. """

	def __init__(self, debug=False):

		print("Initializing Standalone Controller.")

		self.debug = debug

		# Create class members for pipeline processes
		self.dbConn = IoTClient()

	def processImage(imageRGB, imageDepth):

		# Step 1: Detection

		# Step 2: Localization
		detection3D = LocationFromDetection(imageRGB, imageDepth)

		if self.debug:
			print("Localization: ", detection3D)

		# Step 3: Bin Identification
		binStr = findBin(detection3D[0], detection3D[1], detection3D[2])

		if self.debug:
			print("Bin Location: ", binStr)

		# Step 4: Database Connection
		self.dbConn.sendMessage(binStr)

		if self.debug:
			print("Database Message Sent")
			print("Processing pipeline complete.")