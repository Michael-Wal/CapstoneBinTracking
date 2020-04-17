

class WarehouseAisle:
	# Cooridnate Reference:
	# X perpendicular to aisle, Y parallel to aisle, Z upwards


	def __init__(self, aisleNum, aisleWidth=3.0, binWidth=1.0, binHeight=0.3, binDepth=2.0, aisleLength=20.0, aisleHeight=4.0):

		# Aisle number
		self.aisleNum = aisleNum

		# These parameters are used to check if the detection is within this aisle
		self.aisleLength = aisleLength
		self.aisleHeight = aisleHeight
		self.aisleWidth = aisleWidth

		# These parameters are used to determine which bin contains the detection
		self.binWidth = binWidth
		self.binHeight = binHeight
		self.binDepth = binDepth

	def findBin(self, coords):
		""" Find which bin on this aisle is the detection occurring based on the detection's coordinates. """

		# Check the X value to determine which size of the aisle the detection is occurring
		if 0 < abs(coords[0]) - (self.aisleWidth / 2) < self.binDepth:
			# Assign side of aisle depending on sign of X coordinate
			side = "A" if coords[0] < 0 else "B"

		else:
			print("X Coordinate ", coords[0], " is not within a bin.")
			return None

		# Check Section from Z coordinate
		if coords[1] < 0 or coords[1] > self.aisleLength:
			print("Y Coordinate ", coords[1], " is not within the aisle.")
			return None
		else:
			sect = int(coords[1] / self.binWidth)

		# Check Row from Z coordinate
		if coords[2] < 0 or coords[2] > self.aisleHeight:
			print("Z Coordinate ", coords[2], " is not within the aisle.")
			return None
		else:
			row = int(coords[2] / self.binHeight)

		# Construct string from bin location and return
		binStr = "Aisle " + side + str(self.aisleNum) \
			+ ", Section " + str(sect) + ", Row " + str(row)

		return binStr



