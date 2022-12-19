import cv2
import numpy as np
import math
import sys
import time


from Camera_CV import detect_edges,region_of_interest , detect_line_segments , average_slope_intercept
from Camera_CV import make_points,display_lines , display_heading_line , get_steering_angle

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

time.sleep(1)

speed = 25
lastTime = 0
lastError = 0

kp = 0.4
kd = kp * 0.65

while True:

    ret,frame = video.read()

    cv2.imshow("original",frame)
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    cv2.imshow("heading line",heading_image)

    now = time.time()
    dt = now - lastTime

    deviation = steering_angle - 90
    error = abs(deviation)

    derivative = kd * (error - lastError) / dt
    proportional = kp * error
    PD = int(speed + derivative + proportional)
    spd = abs(PD) / 100

    steer = -(deviation / 100)*1.5

    if spd > 0.25:
        spd = 0.25

#    if steer > 0.5:
#        steer = 0.5
#    elif steer < -0.5:
#        steer = -0.5

    lastError = error
    lastTime = time.time()

    #print(steering_angle,dt)





    key = cv2.waitKey(1)
    if key == 27:
        break