#############################################導入函式庫與相關程式檔案
import RPi.GPIO as GPIO
import time
import numpy as np
import cv2
import numpy as np
import math
import sys
import time
import csv

from Camera_CV import detect_edges,region_of_interest , detect_line_segments , average_slope_intercept
from Camera_CV import make_points,display_lines , display_heading_line , get_steering_angle

from Motor_Module import Motor
from EncoderRead import Encoder
from Timer import perpetualTimer
##############################################腳位設定

GPIO.setmode(GPIO.BCM)

motor_FR = Motor(26 ,19, 13)
e1 = Encoder(5, 6)    #A

motor_FL = Motor(21, 20, 16)
e2 = Encoder(18,12)    #B

motor_RR = Motor(22, 27,  17)
e3 = Encoder(11,10)   #C

motor_RL = Motor(23, 24, 25)
e4 = Encoder(14,15)   #D

##############################################參數初始設定
Astep_prev = 0
Bstep_prev = 0
Cstep_prev = 0
Dstep_prev = 0


##############################################
ThetaA_prev = 0
ThetaA_change_prev=0

ThetaB_prev = 0
ThetaB_change_prev=0

ThetaC_prev = 0
ThetaC_change_prev=0

ThetaD_prev = 0
ThetaD_change_prev=0
#############################################
errorA = 0
errorA_prev = 0
inteA = 0
inteA_prev = 0

errorB = 0
errorB_prev = 0
inteB = 0
inteB_prev = 0

errorC = 0
errorC_prev = 0
inteC = 0
inteC_prev = 0

errorD = 0
errorD_prev = 0
inteD = 0
inteD_prev = 0
###############################################

dt = 0.01

PWM = 0
Vmax =  1
Vmin = -1

i = 0 #count
turning_R = 0
turning_L = 0
turning_radians = 0
deviation = 0
pi = 3.142
################################################################
kp = 0.01    #0.01
ki = 0.001
kd = 0.0


################################################################# 數據初始
x_value = 0
total_1 = 0
total_2 = 0
total_3 = 0
total_4 = 0
total_5 = 0
total_6 = 0
total_7 = 0
total_8 = 0

fieldnames = ["Number","deviation","Target R","Target L","FR","RR","FL","RL","turning_radians"]

with open('data.csv', 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()
################################################################
def PID_system():
    global ThetaA_prev ,ThetaB_prev ,ThetaC_prev ,ThetaD_prev
    global ThetaA_change_prev , ThetaB_change_prev ,ThetaC_change_prev ,ThetaD_change_prev
    global inteA_prev  ,inteB_prev  ,inteC_prev  ,inteD_prev
    global errorA_prev ,errorB_prev ,errorC_prev ,errorD_prev

    #Central = 0  #round( 50*(np.sin(2*pi*0.05*i)),0)

    TargetRPM_R =turning_R  #23.33
    TargetRPM_L =turning_L   #60
    Astep = e1.getValue()
    Bstep = e2.getValue()
    Cstep = e3.getValue()
    Dstep = e4.getValue()

    ThetaA =round( (Astep/864),3)
    ThetaB =round( (Bstep/864),3)
    ThetaC =round( (Cstep/864),3)
    ThetaD =round( (Dstep/864),3)

    ThetaA_change =round((ThetaA - ThetaA_prev),3)
    ThetaB_change =round((ThetaB - ThetaB_prev),3)
    ThetaC_change =round((ThetaC - ThetaC_prev),3)
    ThetaD_change =round((ThetaD - ThetaD_prev),3)

    ######################################################
    if TargetRPM_R > 0 :                   #處理重置步數問題
        if ThetaA_change < 0:
            ThetaA_change = ThetaA_change_prev
        #if ThetaB_change < 0 :
        #    ThetaB_change = ThetaB_change_prev
        if ThetaC_change < 0 :
            ThetaC_change = ThetaC_change_prev
        #if ThetaD_change < 0 :
        #    ThetaD_change = ThetaD_change_prev

    elif TargetRPM_R < 0 :
        if ThetaA_change > 0:
            ThetaA_change = ThetaA_change_prev
        #if ThetaB_change > 0:
            ThetaB_change = ThetaB_change_prev
        if ThetaC_change > 0:
            ThetaC_change = ThetaC_change_prev
        #if ThetaD_change > 0:
        #    ThetaD_change = ThetaD_change_prev

    if TargetRPM_L > 0 :                   #處理重置步數問題
        #if ThetaA_change < 0:
        #    ThetaA_change = ThetaA_change_prev
        if ThetaB_change < 0 :
            ThetaB_change = ThetaB_change_prev
        #if ThetaC_change < 0 :
        #    ThetaC_change = ThetaC_change_prev
        if ThetaD_change < 0 :
            ThetaD_change = ThetaD_change_prev

    elif TargetRPM_L < 0 :
        #if ThetaA_change > 0:
        #    ThetaA_change = ThetaA_change_prev
        if ThetaB_change > 0:
            ThetaB_change = ThetaB_change_prev
        #if ThetaC_change > 0:
        #    ThetaC_change = ThetaC_change_prev
        if ThetaD_change > 0:
            ThetaD_change = ThetaD_change_prev
    ######################################################
    rpm_A = round((ThetaA_change/dt)*60  , 3 )
    rpm_B = round((ThetaB_change/dt)*60  , 3 )
    rpm_C = round((ThetaC_change/dt)*60  , 3 )
    rpm_D = round((ThetaD_change/dt)*60  , 3 )

    errorA = round((TargetRPM_R - rpm_A) , 3)
    errorB = round((TargetRPM_L - rpm_B) , 3)
    errorC = round((TargetRPM_R - rpm_C) , 3)
    errorD = round((TargetRPM_L - rpm_D) , 3)

    inteA = round((inteA_prev + dt*(errorA+errorA_prev)/2) , 3)
    inteB = round((inteB_prev + dt*(errorB+errorB_prev)/2) , 3)
    inteC = round((inteC_prev + dt*(errorC+errorC_prev)/2) , 3)
    inteD = round((inteD_prev + dt*(errorD+errorD_prev)/2) , 3)

    V_A = round((kp*errorA + ki*inteA + kd*(errorA - errorA_prev)/dt) , 3)
    V_B = round((kp*errorB + ki*inteB + kd*(errorB - errorB_prev)/dt) , 3)
    V_C = round((kp*errorC + ki*inteC + kd*(errorC - errorC_prev)/dt) , 3)
    V_D = round((kp*errorD + ki*inteD + kd*(errorD - errorD_prev)/dt) , 3)
    ###################################################################################
    if V_A > Vmax:
        V_A = Vmax
        inteA = inteA_prev
    if V_A < Vmin:
        V_A = Vmin
        inteA = inteA_prev

    if V_B > Vmax:
        V_B = Vmax
        inteB = inteB_prev
    if V_B < Vmin:
        V_B = Vmin
        inte_B = inteB_prev

    if V_C > Vmax:
        V_C = Vmax
        inteC = inteC_prev
    if V_C < Vmin:
        V_C = Vmin
        inte_C = inteC_prev

    if V_D > Vmax:
        V_D = Vmax
        inteD = inteD_prev
    if V_D < Vmin:
        V_D = Vmin
        inte_D = inteD_prev
    ####################################################################################

    PWM_A = V_A
    PWM_B = V_B
    PWM_C = V_C
    PWM_D = V_D

    motor_FR.move(PWM_A)
    motor_FL.move(PWM_B)
    motor_RR.move(PWM_C)
    motor_RL.move(PWM_D)

    #print(TargetRPM_R , TargetRPM_L ,"  ", rpm_A ,"  ", rpm_C ,"  ", rpm_B ,"  ", rpm_D )

    ###################################################################
    Astep_prev = Astep
    ThetaA_prev = ThetaA
    ThetaA_change_prev = ThetaA_change
    errorA_prev = errorA
    inteA_prev = inteA

    Bstep_prev = Bstep
    ThetaB_prev = ThetaB
    ThetaB_change_prev = ThetaB_change
    errorB_prev = errorB
    inteB_prev = inteB

    Cstep_prev = Cstep
    ThetaC_prev = ThetaC
    ThetaC_change_prev = ThetaC_change
    errorC_prev = errorC
    inteC_prev = inteC

    Dstep_prev = Dstep
    ThetaD_prev = ThetaD
    ThetaD_change_prev = ThetaD_change
    errorD_prev = errorD
    inteD_prev = inteD



    with open('data.csv', 'a') as csv_file:
        print(x_value," ",deviation," ",TargetRPM_R," ",TargetRPM_L," ",
        rpm_A," ",rpm_C," ",rpm_B," ",rpm_D," ",turning_radians)

        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        info = {
            "Number": x_value,
            "deviation": deviation,
            "Target R": TargetRPM_R,
            "Target L": TargetRPM_L,
            "FR": rpm_A,
            "RR": rpm_C,
            "FL": rpm_B,
            "RL": rpm_D,
            "turning_radians": turning_radians
            }
        csv_writer.writerow(info)

################################################################

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
speed = 0
lastTime = 0
lastError = 0

ckp = 0.1#0.4
ckd = 0 #ckp * 0.65

print("Start")
time.sleep(1)

t = perpetualTimer(dt ,PID_system)
t.start()

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
    cdt = now - lastTime

    deviation = steering_angle - 90
    error = abs(deviation)

    derivative = ckd * (error - lastError) / cdt
    proportional = ckp * error
    PD = int(speed + derivative + proportional)
    #spd = PD / 100

    steer = (deviation / 100)

    #if spd > 0.25:
    #    spd = 0.25
    if steer>0:

        k=math.tan( (90-deviation)*pi/180)
        R=round( 60*(30*k*k-120) / (30*k*k+120)      ,2 )
        turning_radians=round(7.5*k*k,3)
        turning_R = R
        turning_L = 60


    elif steer<0:
        k=math.tan( (90+deviation)*pi/180)
        L=round( 60*(30*k*k-120) / (30*k*k+120)      ,2 )
        turning_radians=round(7.5*k*k,3)
        turning_R = 60
        turning_L = L
    else:
        turning_radians=9999999
        turning_R=60
        turning_L=60


    #print(deviation,turning_R,turning_L,turning_radians)
    '''
    with open('data.csv', 'a') as csv_file:
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        info = {
            "Number": x_value,
            "deviation": deviation,
            "turning_R": turning_R,
            "turning_L": turning_L,
            "turning_radians": turning_radians
        }

        csv_writer.writerow(info)

    '''
    lastError = error
    lastTime = time.time()

    x_value+=1

    key = cv2.waitKey(1)
    if key == 27:
        break