This package is designed to be used with an Intel RealSense D435 camera mounted in an aisle of a warehouse. Using the video stream from the camera, this ananlysis pipeline can detect and localize people within view of the camera and identify when they interact with bins in the aisle. This package was created to be compatible with the Robot Operating System, however that functionality was not included in the final design and is left as potential future development.

This package was created for a Georgia Institute of Technology Senior Capstone project in the Spring semester of 2020.

**Running Instructions**
Before running the program, make sure to have a python3 environment with the required files listed in requirements.txt. The dependencies in this file can be installed automatically with the command:

`pip3 install requirements.txt`

Before starting the application, some parameters need to be set in controllerStandalone. The parameter `TEST_WITH_VIDEO` controls whether a video or single image will be analyzed. The paths to these files can be specified by changing the `pathRGB` and `pathDep` fields. Once the environment is setup and the parameters have been set, the main analysis pipeline can be run by navigating to and running the controller with the following commands.

`
cd /PATH/TO/PACKAGE/CapstoneBinTracking/src/
python3 controllerStandalone.py
`

**Package Structure:**

* /src - Source Files
	* controllerStandalone.py - Standalone Python3 controller for the analsis pipeline. Current functionality performs analysis on prerecorded photos and videos.
	* databaseMessaging.py - This file contains the IoTClient class, which is used to send messages to an external Azure database containing records of each bin interaction.
	* databaseMEssagingNode.py - This file contains a ROS wrapper class for databaseMessaging.py - it's functionality is currently unused but may be useful for future development.
	* findBin.py - Class containing mathematical model of one aisle of a warehouse. This class is used as a coorindate lookup of shelf and bin locations given the 3D coordinates of a detection.
	* localizationNode.py - This file contains a ROS wrapper class for localizationFromDetection.py - it's functionality is currently unused but may be useful for future development.
	* localizationFromDetection.py - This file contains the function LocalizationFromDetection, which combines the 2D bounding box of the neural network detection with the stereo depth frame from the Realsense camera to determine the 3D location of the worker in world coordinates.
	* trackInteractions.py - This files contains the TrackWorkers class, which is used to maintain records of worker detections over time to distinguish picked parts with passing workers.
	* workerDetection.py - This file contains the WorkerDetection class, which uses a ResNet50 neural network to identify workers within each frame passed to it from the camera.

* data/ - Folder to contain data files including Neural network and test videos/images.	
	* resnet50_coco_best_v2.0.1.h5 - ResNet50 Neural Network used for worker detection.
* CMakeLists.txt - ROS Package information, not currently used.
* package.xml - ROS Package information, not currently used.
* msg/ - ROS Message types, not currently used
* requirements.txt - Python Requirements file listing necessary packages for standalone operation
* requirementsROS.txt - Python Requirements file listing packages needed for ROS integration

**Contacts**

Below are the names and contact emails for each of the projects' team members. All emails are @gatech.edu:

* Byrnes, Walker (wbyrnes3)
* Islam, Sanjida (sislam39)
* Reddy, Rohit (rreddy39)
* Safwan, Muhammad (msafwan3)
* Vu, Nhan (nvu31)
* Wali, Omar (omar.wali)
* Waleign, Michael (mwaleign3)