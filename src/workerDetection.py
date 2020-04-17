
from imageai.Detection import ObjectDetection
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
import os


class WorkerDetector:

	def __init__(self, basePath, debug=False):

		# Set debug flag
		self.debug = debug

		# Initialize detector
		self.detector = ObjectDetection()
		self.detector.setModelTypeAsRetinaNet()
    	self.detector.setModelPath( os.path.join(basePath , "data/resnet50_coco_best_v2.0.1.h5"))

    	# Set network to only detect people
    	self.customObjs = self.detector.CustomObjects(person=True)

    	# Available detection speed options: normal, fast, faster, fastest, flash
    	self.detector.loadModel(detection_speed='fast')

    	print("Worker Detector ready.")

    def detectFromImage(self, imageRGB):

    	# Perform detection
    	detImg, detections = self.detector.detectCustomObjectsFromImage(custom_objects=self.customObjs, input_image=imageRGB, 
    		input_type="array", output_type="array")

    	if self.debug:
    		print("Detected ", len(detections), " workers.")
    		plt.imshow(detImg)
    		plt.show()

    	# Extract bounding boxes from detection dictionaries
    	bboxes = []
    	for d in detections:
    		bboxes.append({
    			"x1": d[0],
    			"y1": d[1],
    			"x2": d[2],
    			"y2": d[3],
    			"height": d[3] - d[1],
    			"width": d[2] - d[0]
    			})

    	# Return list of bounding boxes
    	return bboxes


""" Procedural test script for detection
	Author: Muhammad Safwan

"""

#python version 3.6
#tensorflow version 1.15
#install following:
#pip3 install tensorflow==1.15
#pip3 install opencv-python
#pip3 install keras
#pip3 install imageai --upgrade
#most of the code from https://towardsdatascience.com/object-detection-with-10-lines-of-code-d6cb4d86f606
#model from https://github.com/OlafenwaMoses/ImageAI/releases/download/1.0/resnet50_coco_best_v2.0.1.h5

def detectWorker(execution_path):
    detector = ObjectDetection()
    detector.setModelTypeAsRetinaNet()
    detector.setModelPath( os.path.join(execution_path , "resnet50_coco_best_v2.0.1.h5"))
    detector.loadModel()

    detections = detector.detectObjectsFromImage(input_image=os.path.join(execution_path , "6.png"), 
    	output_image_path=os.path.join(execution_path , "image_6.png"))

    for eachObject in detections:
        print(eachObject["name"] , " : " , eachObject["percentage_probability"] )
        print("ObjectKeys: ", eachObject.keys())


# Testing function
if __name__ == "__main__":
    modelPath = "~/catkin_ws/src/CapstoneBinTracking/"
    wd = WorkerDetector(modelPath)

    # Import test image
    imagePath = ""  # TEST IMAGE PATH
    img = np.asarray(Image.open(imagePath))
    detections = wd.detectFromImage(img)

    # Print out detections
    print("Detections: \n", detections)


