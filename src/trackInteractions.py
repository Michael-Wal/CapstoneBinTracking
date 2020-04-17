## This class maintains a dictionary of current interactions, and distinguishes between 
## intentional interactions and passing interactions (false positives).

import datetime as dt


class TrackWorkers:

	def __init__(self, detectionTime=1.0, timeoutTime=5.0, debug=False):

		# This backing dictionary holds the current tracked bins.
		# Key: Bin string
		# Value: List -> (startTime, totalTime, lastCheckin, publishedBool)
		self.currentBins = dict()

		# Save parameters for detection and timeout times
		self.detectionTime = dt.timedelta(seconds=detectionTime)
		self.timeoutTime = dt.timedelta(seconds=timeoutTime)

		self.debug = debug

	def updateDetections(self, detections):
		""" Check current detections against backing dictionary to update with a current frame. """

		currTime = dt.datetime.now()

		for d in detections:

			if d in self.currentBins.keys():
				# Update dictionary values with new detection information
				oldVals = self.currentBins[d]

				# Only worry about updating if we haven't pushed to db yet
				if not oldVals[3]:

					# Update values with new checkin
					td = currTime - oldVals[2]
					self.currentBins[d] = [oldVals[0], oldVals[1] + td, currTime, oldVals[3]]

			else:
				# Create new entry for this bin
				self.currentBins[d] = [currTime, dt.timedelta(), currTime, False]

		# Garbage collection: Remove entries that haven't been seen in a while
		toRemove = [bin for bin in self.currentBins.keys() if currTime - self.currentBins[bin][2] > self.timeoutTime]
		if self.debug:
			print("Removing ", len(toRemove), " timed-out bins.")

		for bin in toRemove:
			self.currentBins.pop(bin)
			if self.debug:
				print("Bin ", bin, " has timed out. ", currTime.isoformat())

	def sendDetections(self):
		""" Return a list of detections that are ready for publish to the database. """

		sendDetections = []
		for bin in self.currentBins.keys():

			# Check if interaction has not yet been published
			if not self.currentBins[bin][3] and self.currentBins[bin][1] > self.detectionTime:

				# Add bin name to return list
				sendDetections.append(bin)

				# Update that bin has been published
				self.currentBins[bin] = self.currentBins[bin][:-1] + [True,]

				if self.debug:
					print("Publishing ", bin, " at ", dt.datetime.now())

		return sendDetections