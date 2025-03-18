# first, import all necessary modules
import cv2
import depthai

# Necessary imports for ArUco detection
import imutils

import time

from Water_pump import Water_pump


def get_lat_long():
    from geocoder import ip
    from datetime import datetime
    g = ip('me')
    # Get the current time in ISO 8601 format
    current_time = datetime.now()
    current_time_iso8601 = current_time.strftime("%H:%M:%SZ")
    if g.latlng:
        return g.latlng[0], g.latlng[1], current_time_iso8601
    else:
        pass

#initialize the water pump shooter
water_shooter = Water_pump()


arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters_create()

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")


# Pipeline tells DepthAI what operations to perform when running - you define all of the resources used and flows here
pipeline = depthai.Pipeline()

# First, we want the Color camera as the output
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)  # 300x300 will be the preview frame size, available as 'preview' output of the node
cam_rgb.setInterleaved(False)


# XLinkOut is a "way out" from the device. Any data you want to transfer to host need to be send via XLink
xout_rgb = pipeline.createXLinkOut()
# For the rgb camera output, we want the XLink stream to be named "rgb"
xout_rgb.setStreamName("rgb")
# Linking camera preview to XLink input, so that the frames will be sent to host
cam_rgb.preview.link(xout_rgb.input)

# The same XLinkOut mechanism will be used to receive nn results
xout_nn = pipeline.createXLinkOut()
xout_nn.setStreamName("nn")
# detection_nn.out.link(xout_nn.input)


# Pipeline is now finished, and we need to find an available device to run our pipeline
# we are using context manager here that will dispose the device after we stop using it
with depthai.Device(pipeline) as device:
    # From this point, the Device will be in "running" mode and will start sending data via XLink

    # To consume the device results, we get two output queues from the device, with stream names we assigned earlier
    q_rgb = device.getOutputQueue("rgb")
    q_nn = device.getOutputQueue("nn")

    # Here, some of the default values are defined. Frame will be an image from "rgb" stream, detections will contain nn results
    frame = None
    detections = []

    splashed_enemies = []

    # Main host-side application loop
    while True:
        # we try to fetch the data from nn/rgb queues. tryGet will return either the data packet or None if there isn't any
        in_rgb = q_rgb.tryGet()
        in_nn = q_nn.tryGet()

        if in_rgb is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            frame = in_rgb.getCvFrame()

        if in_nn is not None:
            # when data from nn is received, we take the detections array that contains mobilenet-ssd results
            detections = in_nn.detections


        if frame is not None:
            frame = imutils.resize(frame, width=1000)

            # detect ArUco markers in the input frame
            (corners, ids, _) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
            # (corners, ids, rejected)

            # verify *at least* one ArUco marker was detected
            if len(corners) > 0:
		        # flatten the ArUco IDs list
                ids = ids.flatten()
		        # loop over the detected ArUCo corners
                for markerID in ids:
                    if (markerID == 11):
                        water_shooter.stop_shooting()
                        print("Friendly detcted")
                    else:
                        if markerID not in splashed_enemies:
                            water_shooter.shoot()
                            splashed_enemies.append(markerID)
                            initial_time = time.time()
                            while (time.time() - initial_time <= 2 ):
                                pass
                            water_shooter.stop_shooting()
                            # lat, lon, time_v = get_lat_long()
                            #print(f"RTXDC_2024_UPRM_LIDRON_UAV_WaterBlast!_{markerID}_{time_v}_{lat}_{lon}")
                        else:
                            water_shooter.stop_shooting()


            
        key = cv2.waitKey(1) & 0xFF
	    # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
