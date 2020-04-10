## This ROS node is a wrapper for locationFromDetection.py, allowing it to be used in a
## ROS-based processing pipeline. 
## AUTHOR: Walker Byrnes
## EMAIL: walkerbyrnes@gmail.com

import rospy
from sensor_msgs import Image
from std_msgs import Header
from geometry_msgs import Point, PointStamped

import locationFromDetection

class LocalizationNode:

	def __init__(camOff=(0, 0, 0)):
		
		# Initialize node
		rospy.init_node("LocalizationNode")

		# Setup publishers and subscribers
		self.subHandler = rospy.Subscriber("cam_detection", StereoDetection, self.handleDetection)
		self.pubHandler = rospy.Publisher("cam_worker_location", PointStamped, 10)

		# Initialize class variables
		self.camOffset = camOff

		# Set shutdown callback function
		rospy.on_shutdown(self.onDestroy)

	def handleDetection(self, msg):

		# StereoDetection message definition
		# rgbImage: sensor_msgs.Image
		# depthImage: sensor_msgs.Image
		# x1: int64  # Bounding box coordinates
		# x2: int64
		# y1: int64
		# y2: int64

		# Construct detection dictionary
		det = {
	        "x1": msg.x1,
	        "y1": msg.x2,
	        "x2": msg.y1,
	        "y2": msg.y2,
	        "height": abs(msg.y2 - msg.y1),
	        "width": abs(msg.x2 - msg.x1)
    	}

    	detection3D = locationFromDetection(msg.rgbImage, msg.depthImage, det, cameraLocationOffset=self.camOffset)

    	# Copy message header from rgb image
    	retMsgHead = msg.rgbImage.header

    	# Construct Point message
    	retMsgPoint = Point()
    	retMsgPoint.x = detection3D[0]
    	retMsgPoint.y = detection3D[1]
    	retMsgPoint.z = detection3D[2]

    	retMsg = PointStamped()
    	retMsg.header = retMsgHead
    	retMsg.point = retMsgPoint

    	# Publish stamped localization
    	self.pubHandler.publish(retMsg)

    def onDestroy(self):

    	print("Shutting down LocalizationNode...")


# Initialize node
if __name__ == "__main__":

	print("Beginning node setup")
	ln = LocalizationNode()
	print("Node initialization complete")

	rospy.spin()
