import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Motor:
    def __init__(self, EnaA, In1A, In2A):
        self.EnaA = EnaA
        self.In1A = In1A
        self.In2A = In2A

        GPIO.setup(self.EnaA, GPIO.OUT);
        GPIO.setup(self.In1A, GPIO.OUT);
        GPIO.setup(self.In2A, GPIO.OUT)

        self.pwmA = GPIO.PWM(self.EnaA, 1000);
        self.pwmA.start(0);

    def move(self, speed): #轉速pwm輸出
        speed *= 100

        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100

        self.pwmA.ChangeDutyCycle(abs(speed))
        if speed > 0:
            GPIO.output(self.In1A, GPIO.LOW);
            GPIO.output(self.In2A, GPIO.HIGH)
        elif speed < 0 :
            GPIO.output(self.In1A, GPIO.HIGH);
            GPIO.output(self.In2A, GPIO.LOW)
        else :
            GPIO.output(self.In1A, GPIO.LOW);
            GPIO.output(self.In2A, GPIO.LOW)

    def stop(self, t=0):
        self.pwmA.ChangeDutyCycle(0);
        self.mySpeed = 0
        time.sleep(t)


'''

k = 0.2
a = 0.4
b = 0.6

#####################EnaA, In1A, In2A

motor_FR = Motor(26, 19, 13)     #A
motor_FL = Motor(21, 20, 16)     #B
motor_RR = Motor(22, 27, 17)     #C
motor_RL = Motor(23, 24, 25)     #D

print("Start")
motor_FR.move(0)
motor_FL.move(0)
motor_RR.move(0)
motor_RL.move(0)
time.sleep(3)

while True:



    print("go")
    motor_FR.move(k)
    motor_FL.move(k)
    motor_RR.move(k)
    motor_RL.move(k)
    time.sleep(3)


    motor_FR.move(0)
    motor_FL.move(0)
    motor_RR.move(0)
    motor_RL.move(0)
    time.sleep(1)

    motor_FR.move(a)
    motor_FL.move(a)
    motor_RR.move(a)
    motor_RL.move(a)
    time.sleep(3)

    motor_FR.move(0)
    motor_FL.move(0)
    motor_RR.move(0)
    motor_RL.move(0)
    time.sleep(1)

    motor_FR.move(b)
    motor_FL.move(b)
    motor_RR.move(b)
    motor_RL.move(b)
    time.sleep(3)

    motor_FR.move(0)
    motor_FL.move(0)
    motor_RR.move(0)
    motor_RL.move(0)
    time.sleep(3)


    print("back")
    motor_FR.move(-k)
    motor_FL.move(-k)
    motor_RR.move(-k)
    motor_RL.move(-k)
    time.sleep(3)


    motor_FR.move(0)
    motor_FL.move(0)
    motor_RR.move(0)
    motor_RL.move(0)
    time.sleep(1)

    motor_FR.move(-a)
    motor_FL.move(-a)
    motor_RR.move(-a)
    motor_RL.move(-a)
    time.sleep(3)

    motor_FR.move(0)
    motor_FL.move(0)
    motor_RR.move(0)
    motor_RL.move(0)
    time.sleep(1)

    motor_FR.move(-b)
    motor_FL.move(-b)
    motor_RR.move(-b)
    motor_RL.move(-b)
    time.sleep(3)
    print("done")

'''