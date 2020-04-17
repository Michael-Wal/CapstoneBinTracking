

import numpy as np
from PIL import Image

# Raspberry Pi camera module focal length: 3.6mm
# Source: https://raspberrypi.stackexchange.com/questions/81964/calculating-focal-length-of-raspberry-pi-camera
CAMERA_FOCAL_LENGTH_PX = 2571

def LocationFromDetection(imageRGB: Image, imageDepth: Image, detection, cameraLocationOffset=None, stereo=True, debug=False, inspect=False):

    # If debug is enabled, show rgb and depth images for inspection
    if inspect:
        ShowDetection(imageRGB, imageDepth, detection)

    # Extract image plane location from detection
    detectionX_px = detection["x1"] + (0.5 * detection["width"])
    detectionY_px = detection["y1"] + (0.5 * detection["height"])

    # Assume camera aperture center is aligned properly with image center
    # Calculate u,v from center of camera
    u0_px = int(imageRGB.size[1] / 2)
    v0_px = int(imageRGB.size[0] / 2)
    u_px = detectionX_px - u0_px
    v_px = detectionY_px - v0_px

    # Calculate depth of detection, either through stereo vision or size estimation
    if stereo:
        # If rgb and depth images are of different sizes, convert lookup coordinates between them
        depthX_px = (detectionX_px / imageRGB.size[1]) * imageDepth.size[1]
        depthY_px = (detectionY_px / imageRGB.size[0]) * imageDepth.size[1]

        # convert detection coordinates to depth image size
        depthDetection = detection
        depthDetection["x1"] = depthX_px
        depthDetection["y1"] = depthY_px
        depthDetection = ConvertDetectionCoords(imageRGB.size, imageDepth.size, detection)
        depthX_px = depthDetection["x1"]
        depthY_px = depthDetection["y1"]

        # Use stereo depth lookup to calculate 3D detection coordinate
        Z_m = np.mean(imageDepth.getpixel((depthY_px, depthX_px)))  # Assume depth is normal to camera plane
        print("Z Depth: ", Z_m)

    else:
        # Use height of detection to estimate distance of detection
        h_px = detection.height
        H_m = 1.753  # m, average height of American adult

        # Using pinhole camera model & properties of similar triangles
        # Depth Z = H * f / h
        Z_m = H_m * CAMERA_FOCAL_LENGTH_PX / h_px

    # Use u, v, and Z to determine X and Y coordinates in camera space
    # X = Z * u/f    |    Y = Z * u/f
    X_m = Z_m * u_px / CAMERA_FOCAL_LENGTH_PX
    Y_m = Z_m * v_px / CAMERA_FOCAL_LENGTH_PX

    # Combine components into camera frame location of worker
    workerLocationCamera = [X_m, Y_m, Z_m, 1]
    if debug:
        print("Detected worker in camera frame:\n", workerLocationCamera)

    # Transform detection from camera frame into world frame
    if cameraLocationOffset is not None:
        workerLocationWorld = np.dot(cameraLocationOffset, workerLocationCamera)
    else:
        workerLocationWorld = workerLocationCamera

    return workerLocationWorld


def ShowDetection(imageRGB, imageDepth, detection):
    # Use pyplot to show image frames to viewer to confirm detection

    # Only import pyplot if needed
    import matplotlib.pyplot as plt
    from matplotlib import patches

    # Show rgb image with detection outline
    fig, ax = plt.subplots()

    ax.imshow(imageRGB)

    ax.add_patch(patches.Rectangle(
        (detection["x1"], detection["y1"]),
        detection["width"],
        detection["height"],
        edgecolor = 'blue',
        facecolor = 'red',
        fill=False
    ) )

    plt.show()

    print("ImageRGB: ", imageRGB.size)
    print("ImageDep: ", imageDepth.size)

    # Show depth image with equivalent rectangle
    depthDetection = ConvertDetectionCoords(imageRGB.size, imageDepth.size, detection)
    fig, ax = plt.subplots()

    ax.imshow(imageDepth)

    ax.add_patch(patches.Rectangle(
        (depthDetection["x1"], depthDetection["y1"]),
        depthDetection["width"],
        depthDetection["height"],
        edgecolor = 'blue',
        facecolor = 'red',
        fill=False
    ) )

    plt.show()


def ConvertDetectionCoords(size1, size2, detection):
    "Transfrom a detection in image of size size1 to fit an image with size2 dimensions."
    xScale = size2[0] / size1[0]
    yScale = size2[1] / size1[1]

    newDetection = {
        "x1": detection["x1"] * xScale,
        "x2": detection["x2"] * xScale,
        "y1": detection["y1"] * yScale,
        "y2": detection["y2"] * yScale,
        "height": detection["height"] * yScale,
        "width": detection["width"] * xScale
    }

    return newDetection


# Unit Test: AGCO Warehouse Test 2 video frame 25
# Assume correct focal length, ballpark should suffice
if __name__ == "__main__":

    rgbImage = Image.open("./data/test2frame0025color.jpg")
    depthImage = Image.open("./data/test2frame0025depth.jpg")

    print("RGB: ", rgbImage.size)
    print("Depth: ", depthImage.size)

    corners = [(372, 200), (640,714,)]  # Format x,y
    imgPos = ((corners[1][0] + corners[0][0]) / 2,
              (corners[1][1] + corners[0][1]) / 2)
    detection = {
        "x1": corners[0][0],
        "y1": corners[0][1],
        "x2": corners[1][0],
        "y2": corners[1][1],
        "height": corners[1][1] - corners[0][1],
        "width": corners[1][0] - corners[0][0]
    }
    resLoc = LocationFromDetection(rgbImage, depthImage, detection, debug=True)
    print("Worker Location: ", resLoc)

