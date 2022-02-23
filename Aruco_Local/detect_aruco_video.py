'''
Sample Command:-
python3 detect_aruco_video.py
'''

import numpy as np
from utils import ARUCO_DICT, aruco_display
import argparse
import time
import cv2
import sys
import pyrealsense2 as rs

# Configure depth and color streams
#IMU: 923322071214
pipeline1 = rs.pipeline()
config1 = rs.config()
config1.enable_device('923322071214')
config1.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Configure depth and color streams
#Non-IMU: 814412070147
pipeline2 = rs.pipeline()
config2 = rs.config()
config2.enable_device('814412070147')
config2.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline1.start(config1)
pipeline2.start(config2)

time.sleep(1.0)

arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_5X5_100"])
arucoParams = cv2.aruco.DetectorParameters_create()

while True:

	frames1 = pipeline1.wait_for_frames()
	depth_frame1 = frames1.get_depth_frame()
	color_frame1 = frames1.get_color_frame()

	frames2 = pipeline2.wait_for_frames()
	depth_frame2 = frames2.get_depth_frame()
	color_frame2 = frames2.get_color_frame()

	if not depth_frame1 or not color_frame2 or not depth_frame2 or not color_frame2:
		continue

	color_image1 = np.asanyarray(color_frame1.get_data())
	gray1 = cv2.cvtColor(color_image1, cv2.COLOR_BGR2GRAY)

	color_image2 = np.asanyarray(color_frame2.get_data())
	gray2 = cv2.cvtColor(color_image2, cv2.COLOR_BGR2GRAY)

	corners1, ids1, rejected1 = cv2.aruco.detectMarkers(gray1, arucoDict, parameters=arucoParams)
	corners2, ids2, rejected2 = cv2.aruco.detectMarkers(gray2, arucoDict, parameters=arucoParams)

	detected_markers1 = aruco_display(corners1, ids1, rejected1, gray1)
	detected_markers2 = aruco_display(corners2, ids2, rejected2, gray2)

	depth_point = []

	if(len(corners1) > 0):
		depth_intri = depth_frame1.profile.as_video_stream_profile().intrinsics

		x_sum = corners1[0][0][0][0] + corners1[0][0][1][0] + corners1[0][0][2][0] + corners1[0][0][3][0]
		y_sum = corners1[0][0][0][1] + corners1[0][0][1][1] + corners1[0][0][2][1] + corners1[0][0][3][1]
    
		x_centerPixel = x_sum*.25
		y_centerPixel = y_sum*.25

		color_point = [x_centerPixel, y_centerPixel]

		x_int = int(x_centerPixel)
		y_int = int(y_centerPixel)

		depth_value = depth_frame1.get_distance(x_int, y_int)

		if(depth_value == 0):
			continue

		depth_point = rs.rs2_deproject_pixel_to_point(depth_intri, color_point, depth_value)
		depth_point.append(1)
		print(depth_point)

	elif(len(corners2) > 0):
		depth_intri = depth_frame2.profile.as_video_stream_profile().intrinsics

		x_sum = corners2[0][0][0][0] + corners2[0][0][1][0] + corners2[0][0][2][0] + corners2[0][0][3][0]
		y_sum = corners2[0][0][0][1] + corners2[0][0][1][1] + corners2[0][0][2][1] + corners2[0][0][3][1]
    
		x_centerPixel = x_sum*.25
		y_centerPixel = y_sum*.25

		color_point = [x_centerPixel, y_centerPixel]

		x_int = int(x_centerPixel)
		y_int = int(y_centerPixel)

		print(x_int)
		print(y_int)


		depth_value = depth_frame2.get_distance(x_int, y_int)

		if(depth_value == 0):
			continue

		depth_point = rs.rs2_deproject_pixel_to_point(depth_intri, color_point, depth_value)
		depth_point.append(2)
		print(depth_point)

	images = np.hstack((detected_markers2, detected_markers1))
	cv2.imshow("Image", images)

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
	    break

cv2.destroyAllWindows()
pipeline1.stop()
pipeline2.stop()