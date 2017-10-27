import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

Servo1 = 36
Servo2 = 38
Servo3 = 40

GPIO.setup(Servo1, GPIO.OUT)
GPIO.setup(Servo2, GPIO.OUT)
GPIO.setup(Servo3, GPIO.OUT)

pwm_frequency = 50

def set_duty_cycle(angle):
    pulse = 2*float(angle)/180.0+0.5
    duty = 0.1*pulse*pwm_frequency
    #duty = 2.5 + 0.12*float(angle) for frequency of 100
    return duty

pwm_servo1 = GPIO.PWM(Servo1, pwm_frequency)
pwm_servo2 = GPIO.PWM(Servo2, pwm_frequency)
pwm_servo3 = GPIO.PWM(Servo3, pwm_frequency)

##angle = 90
##pwm_servo1.start(set_duty_cycle(angle))
##pwm_servo2.start(set_duty_cycle(angle))
##pwm_servo3.start(set_duty_cycle(angle))

if __name__ == '__main__':
    try:
        while True:

            #90 = threshold clockwise (slowest)
            #91 = threshold counterclockwise (slowest)
            #0 = fastest clockwise
            #180 = fastest counterclockwise
            #Speed actually has nothing to do with angles:
            #Integers can be put directly into .start()
            #Thresholds for .start(): 0, 100
            angle1 = 360
            angle2 = 180
            angle3 = 0
            pwm_servo1.start(set_duty_cycle(angle1))
            pwm_servo2.start(set_duty_cycle(angle2))
            pwm_servo3.start(50)
            print("45")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Program stopped by user")
        GPIO.cleanup()
