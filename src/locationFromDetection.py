## This function calculates the location of the worker in world coordinates from a detection 
## in camera coordinates. ptions exist to either use stereo vision or mono vision with depth 
## from height estimate.
## AUTHOR: Walker Byrnes
## EMAIL: walkerbyrnes@gmail.com

# Raspberry Pi camera module focal length: 3.6mm
# Source: https://raspberrypi.stackexchange.com/questions/81964/calculating-focal-length-of-raspberry-pi-camera
CAMERA_FOCAL_LENGTH_PX = 2571

def LocationFromDetection(imageVec, detection, stereo=FALSE):

# Extract image plane location from detection
    detectionX_px = detection.x
    detectionY_px = detection.y

    # Calculate depth of detection, either through stereo vision or size estimation
    if stereo:
        #TODO: Implement stereo depth estimation

    else:
        
        h_px = detection.height
        H_m = 1.753  # m, average height of American adult

        # Using pinhole camera model & properties of similar triangles
        # Depth Z = H * f / h
        Z_m = H_m * CAMERA_FOCAL_LENGTH_PX / h_px

        # Assume camera aperture center is aligned properly with image center
        # Calculate u,v from center of camera
        u0_px = int(imageVec[0].shape[1] / 2)
        v0_px = int(imageVec[0].shape[0] / 2)
        u_px = u0_px - detectionX_px
        v_px = v0_px - detectionY_px

        # Use u, v, and Z to determine X and Y coordinates in camera space
        # X = Z * u/f    |    Y = Z * u/f
        X_m = Z_m * u_px / detectionX_px
        Y_m = Z_m * v_px / detectionY_px

        # Add camera transform to get position in world space
        workerLocationCamera = [X_m, Y_m, Z_m]
        # TODO: Calculate and apply camera transformation matrix

        workerLocationWorld = workerLocationCamera  # Temporary, assumes camera at origin

        return workerLocationWorld


