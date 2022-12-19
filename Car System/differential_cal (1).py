import math
import time
pi =3.1426

def rpm_R(theta):
    L=60
    k=math.tan( (90-theta)*pi/180)
    R=round(60*(k*k-3)/(k*k+5),3)
    print(theta,L,R)
    time.sleep(0.1)

def rpm_L(theta):
    R=60
    k=math.tan( (90-theta)*pi/180)
    L=round(60*(k*k-3)/(k*k+5),3)
    print(theta,L,R)
    time.sleep(0.1)

############################################

while True:
    for i in range (-90,90):
        print (i)
        if i < 0:
            rpm_L(-i)
        elif i>0:
            rpm_R(i)
        else :
            print("L R = 0")
    break;