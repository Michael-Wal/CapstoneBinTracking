#!/usr/bin/env python3
## This ROS node is a wrapper for databaseMessaging.py, allowing to be used with the ROS messaging framework.
## AUTHOR: Walker Byrnes (walkerbyrnes@gmail.com)

print("Beginning imports")
import rospy
import std_msgs
from geometry_msgs.msg import Point, PointStamped
import locationFromDetection
from capstone_bin_tracking.msg import StereoDetection
from databaseMessaging import IoTClient
print("Imports complete")


class DatabaseMessagingNode:

    def __init__(self, debug=False):

        # Initialize node
        print("Creating database messaging node")
        rospy.init_node("DatabaseMessagingNode")

        # Setup publishers and subscribers
        self.subHandler = rospy.Subscriber("bin_interactions", std_msgs.msg.String, self.handleMessage)

        # Initialize class variables
        self.dbPublisher = IoTClient()
        self.debug = debug

        # Set shutdown callback function
        rospy.on_shutdown(self.onDestroy)
        print("Node ready!")

    def handleMessage(self, msg):

        # Extract message information
        binStr = msg.data

        # Send detection string to database publisher
        self.dbPublisher.sendMessage(binStr)

        if self.debug:
            print("Message ", binStr, " published successfully")

    def onDestroy(self):

        print("Shutting down Localization Node...")
        self.dbPublisher.onDestroy()


# Initialize node
if __name__ == "__main__":

    print("Beginning node setup")
    dmn = DatabaseMessagingNode()
    print("Node initialization complete")

    rospy.spin()
