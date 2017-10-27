#Libraries
import RPi.GPIO as GPIO
import time

#GPIO Mode
GPIO.setmode(GPIO.BOARD)


#set GPIO Pins
Bin1 = 11
Bin2 = 13
Bpwm = 15

slow = 20
med = 50
fast = 100

GPIO.setup(Bin1,GPIO.OUT)
GPIO.setup(Bin2,GPIO.OUT)
GPIO.setup(Bpwm,GPIO.OUT)

rightTopPWM = GPIO.PWM(Bin1,slow)
rightTopPWM.ChangeFrequency(100)
rightTopPWM.start(0)
rightBottomPWM = GPIO.PWM(Bin2,slow)
rightBottomPWM.ChangeFrequency(100)
rightBottomPWM.start(0)

#PWM Parameters

def forward(speed=med):
    rightTopPWM.ChangeDutyCycle(speed)
    rightBottomPWM.ChangeDutyCycle(0)
def reverse(speed=med):
    rightTopPWM.ChangeDutyCycle(0)
    rightBottomPWM.ChangeDutyCycle(speed)
def stop():
    rightTopPWM.ChangeDutyCycle(0)
    rightBottomPWM.ChangeDutyCycle(0)
if __name__ == '__main__':
    try:
        while True:
            forward(slow)
            time.sleep(2)
            reverse(med)
            time.sleep(2)
            forward(fast)
            time.sleep(2)
            stop()
            time.sleep(2)
            

    except KeyboardInterrupt:
        stop()
        print("program stopped")
        GPIO.cleanup()
        
