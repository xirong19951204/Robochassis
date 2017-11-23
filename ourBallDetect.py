import cv2
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import RPi.GPIO as GPIO
import time
from motorRun import *
from ultrasonic import *
from servo import *

def nothing(x):
    pass
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(3)

h, s, v = 100, 100, 100
img_low = np.zeros((15,512,3),np.uint8)

#set GPIO Pins

GPIO_Servo = 40
GPIO.setup(GPIO_Servo,GPIO.OUT)
GPIO_Servo2 = 38
GPIO.setup(GPIO_Servo2,GPIO.OUT)

pwm_frequency = 50

def set_duty_cycle(angle):
    pulse = ((2*float(angle))/180.0) + 0.5
    duty = 0.1*pulse*pwm_frequency
    #duty = 2.5 + 0.12*float(angle) for frequency of 100
    return duty

pwm_servo = GPIO.PWM(GPIO_Servo, pwm_frequency)
pwm_servo2 = GPIO.PWM(GPIO_Servo2, pwm_frequency)

angle = 90

pwm_servo.start(set_duty_cycle(angle))
pwm_servo2.start(set_duty_cycle(angle))
#PWM Parameters
def countPix(img):
    lefthalf = img[0:480,0:250]
    midhalf= img[0:480,250:390]
    righthalf = img[0:480,390:640]
    cv2.imshow('result',lefthalf)
    cv2.imshow('result1',midhalf)
    cv2.imshow('result2',righthalf)
    numWhite1=cv2.countNonZero(lefthalf)
    numWhite3=cv2.countNonZero(midhalf)
    numWhite2=cv2.countNonZero(righthalf)
    #print(str(numWhite1)+ "and" + str(numWhite3) + "and" + str(numWhite2))
    return [numWhite1, numWhite3, numWhite2]

def takePic():
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        cv2.imshow('result', img_low)
        #modify values below
        img_low[:] = [0,136,141]
        # define the range of the blue color in hsv
        lower_green = np.array([0,136,141])
        upper_green = np.array([30,255,255])

        # Threshold the hsv image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        #Bitwise AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        numpix=countPix(mask)
        
        # show the frame
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", mask)
        cv2.imshow("Res", res)
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

        return numpix

def countBluePix(img):
    bluelefthalf = img[0:480,0:320]
    bluerighthalf = img[0:480,320:640]
    cv2.imshow('result',bluelefthalf)
    cv2.imshow('result1',bluerighthalf)
    numBlueWhite1=cv2.countNonZero(bluelefthalf)
    numBlueWhite2=cv2.countNonZero(bluerighthalf)
    #print(str(numBlueWhite1)+ "and" + str(numBlueWhite2))
    return [numBlueWhite1, numBlueWhite2]

def takeBluePic():
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        cv2.imshow('result', img_low)
        #modify values below
        img_low[:] = [105,74,0]
        # define the range of the blue color in hsv
        lower_green = np.array([105,74,0])
        upper_green = np.array([255,255,255])

        # Threshold the hsv image to get only blue colors
        mask = cv2.inRange(hsv, lower_green, upper_green)

        #Bitwise AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        numBluepix=countBluePix(mask)
        
        # show the frame
        cv2.imshow("Frame", image)
        cv2.imshow("Mask", mask)
        cv2.imshow("Res", res)
        key = cv2.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            cv2.destroyAllWindows()
            break

        return numBluepix
def openservo():
    angle1 = 0
    angle2 = 180
    pwm_servo.start(set_duty_cycle(angle1))
    pwm_servo2.sart(set_duty_cycle(angle2))
    print("Open")
    time.sleep(1)

    
def closeservo():
    angle1 = 90
    angle2 = 90
    pwm_servo.start(set_duty_cycle(angle1))
    pwm_servo2.sart(set_duty_cycle(angle2))
    print("close")
    time.sleep(1)
    
try:
    start = time.time()
    stoptime = time.time()
    found = False
    
    while True:
        speed = 25
        pix=takePic()
        #0 left
        #1 middle
        #2 right
        print('left:' + str(pix[0]))
        print('middle:' + str(pix[1]))
        print('right:' + str(pix[2]))
        print('distance:' + str(distance()))
        if pix[1]<30 and not found:
            print(time.time()-start)
            firstright(35)
            if time.time()-start > .15:
                start = time.time()
            stoptime = time.time()
            while time.time()-stoptime < .75:
                stop()
        if pix[1]>30:
            found = True
            
##        if pix[1]<10:
##            right()
##            pix=takePic()
        if pix[1]<pix[0] or pix[1] < pix[2]:
            if pix[0]<pix[2]:
                right(speed)
            elif pix[2]<pix[0]:
                left(speed)
    
        elif distance()>10.0 and pix[1]>pix[0] and pix[1]>pix[2]:
            forward(speed)
            print(distance())
        elif -1<distance()<8:
            forward(50)
            time.sleep(.2)
            stop()
            print("Francesco stopped")
            closeservo()
            break
    found = False
    while True:
        speed = 25
        pix=takeBluePic()
        #0 left
        #1 middle
        #2 right
        print('left:' + str(pix[0]))
        print('middle:' + str(pix[1]))
        print('right:' + str(pix[2]))
        print('distance:' + str(distance()))
        if pix[1]<30 and not found:
            print(time.time()-start)
            firstright(35)
            if time.time()-start > .15:
                start = time.time()
            stoptime = time.time()
            while time.time()-stoptime < .75:
                stop()
        if pix[1]>30:
            found = True
            
##        if pix[1]<10:
##            right()
##            pix=takePic()
        if pix[1]<pix[0] or pix[1] < pix[2]:
            if pix[0]<pix[2]:
                right(speed)
            elif pix[2]<pix[0]:
                left(speed)
    
        elif distance()>10.0 and pix[1]>pix[0] and pix[1]>pix[2]:
            forward(speed)
            print(distance())
        elif -1<distance()<8:
            backward(50)
            time.sleep(1)
            openservo()
            forward(100)
            time.sleep(.5)
            stop()
            print("Francesco stopped")
            break    
except KeyboardInterrupt:
    print("Program stopped by User")
    GPIO.cleanup()

'''
try:
    while True:
        bluepix=takeBluePic()
        if bluepix[0]+bluepix[1]<120:
            right()
        elif bluepix[0]-bluepix[1]>50:
            left()
        elif bluepix[1]-bluepix[0]>50:
            right()
        else:
            stop()
            angle = 180
            pwm_servo.start(set_duty_cycle(angle))
            print ("180")
            time.sleep(1)
            angle = 0
            pwm_servo.start(set_duty_cycle(angle))
            print ("0")
            time.sleep(1)
            print("end of execution")
            break
            
except KeyboardInterrupt:
    print("Program stopped by User")
    GPIO.cleanup()
'''
