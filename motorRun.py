import RPi.GPIO as GPIO
import time

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)
     
# set GPIO Pins
GPIO_Bin1 = 32
GPIO_Bin2 = 36
GPIO_Bpwm = 38
GPIO_Apwm = 12
GPIO_Ain2 = 16
GPIO_Ain1 = 22

slow = 30
med = 50
fast = 100
# Set PWM parameters
#pwm_frequency = 100

def set_duty_cycle(speed):
    pulse =  2*float(speed)
    duty = 0.1*pulse*pwm_frequency
    #duty = 2.5 + 0.12*float(angle) #for frequency of 100
    return duty

GPIO.setwarnings(False)

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)

# set speed to HIGH
##GPIO.output(GPIO_Bpwm, True)
##GPIO.output(GPIO_Bin1, True)
##GPIO.output(GPIO_Bin2, True)
##GPIO.output(GPIO_Apwm, True)
##GPIO.output(GPIO_Ain1, True)
##GPIO.output(GPIO_Ain2, True)

# set PWM Configurations to SLOW
##pwm_AmotorTop = GPIO.PWM(GPIO_Ain1,slow)
##pwm_AmotorTop.ChangeFrequency(100)
##pwm_AmotorTop.start(0)
##pwm_AmotorBottom = GPIO.PWM(GPIO_Ain2, slow)
##pwm_AmotorBottom.ChangeFrequency(100)
##pwm_AmotorBottom.start(0)
##
##pwm_BmotorTop = GPIO.PWM(GPIO_Bin1,slow)
##pwm_BmotorTop.ChangeFrequency(100)
##pwm_BmotorTop.start(0)
##pwm_BmotorBottom = GPIO.PWM(GPIO_Bin2, slow)
##pwm_BmotorBottom.ChangeFrequency(100)
##pwm_BmotorBottom.start(0)

pwm_rightmotor = GPIO.PWM(GPIO_Apwm,slow)
pwm_rightmotor.ChangeFrequency(100)
pwm_rightmotor.start(0)

pwm_leftmotor = GPIO.PWM(GPIO_Bpwm,slow)
pwm_leftmotor.ChangeFrequency(100)
pwm_leftmotor.start(0)


def forward(speed):
    
    pwm_rightmotor.ChangeDutyCycle(speed)
    pwm_leftmotor.ChangeDutyCycle(speed)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    print ("Forward")

def backward(speed):
    
    pwm_rightmotor.ChangeDutyCycle(speed)
    pwm_leftmotor.ChangeDutyCycle(speed)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    print ("Backward")

def stop():
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, False)
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, False)
    print ("Stop")

def right(speed):
    
    pwm_rightmotor.ChangeDutyCycle(speed)
    pwm_leftmotor.ChangeDutyCycle(speed)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    time.sleep(.25)
    stop()
    print ("Right")
    
def left(speed):
    
    pwm_rightmotor.ChangeDutyCycle(speed)
    pwm_leftmotor.ChangeDutyCycle(speed)
    GPIO.output(GPIO_Bin1, False)
    GPIO.output(GPIO_Bin2, True)
    GPIO.output(GPIO_Ain1, True)
    GPIO.output(GPIO_Ain2, False)
    time.sleep(.25)
    stop()
    print ("Left")

def firstright(speed):
    
    pwm_rightmotor.ChangeDutyCycle(speed)
    pwm_leftmotor.ChangeDutyCycle(speed)
    GPIO.output(GPIO_Bin1, True)
    GPIO.output(GPIO_Bin2, False)
    GPIO.output(GPIO_Ain1, False)
    GPIO.output(GPIO_Ain2, True)
    time.sleep(.25)
    print ("Looking for ball")

'''try:
    while True:
        firstright(slow)


except KeyboardInterrupt:
    stop()
    print("Franci stopped")
    GPIO.cleanup()
'''  

