import gpiozero
import time
import RPi.GPIO as GPIO
import numpy as np
#setup pins
GPIO.setmode(GPIO.BCM)
#Set GPIO pins as outputs
GPIO.setup(24, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
#Direction(Forward) = [left forward GPIO pin, right forward GPIO pin]
dir1 = [gpiozero.OutputDevice(18), gpiozero.OutputDevice(26)] #Forward
#Direction(Backward) = [left backward GPIO pin, right backward GPIO pin]
dir2 = [gpiozero.OutputDevice(23), gpiozero.OutputDevice(19)] #Backward
#PWM pins = [left pwm pin, right pwm pin]
pwm = [GPIO.PWM(24, 100), GPIO.PWM(13, 100)]  #PWM

#Initialise pwm at 0
pwm[0].start(0)
pwm[1].start(0)
#Multipliers for uneven motor power
m1mult = 1.0 #Left motor multiplier
m2mult = 0.9 #Right motor multiplier

def motor(mot, value):
  np.clip(value, -100, 100)
  #Drive motor(which motor, PWM intensity)
  if (value > 0):
    #if value is positive: drive forward
    dir1[mot].on()
    dir2[mot].off()
  elif (value < 0):
    #if value is negative: drive backward
    dir1[mot].off()
    dir2[mot].on()
  #set the pwm pin at index mot to "value"
  pwm[mot].ChangeDutyCycle(abs(value))

def drive(magnitude, rotation, tsec):
  motor(0, magnitude - rotation)  #set motor at index 0 (left motor) to (value-rotation)
  motor(1, magnitude + rotation)  #set motor at index 1 (right motor) to (value+rotation)
  #rotation is positive CCW from north
  time.sleep(tsec) #drive the motors for time = tsec
  #then stop
  pwm[0].stop() #stop the pwm pin at index 0 (left motor)
  pwm[1].stop() #stop the pwm pin at index 1 (right motor)

drive(40,0,5)
#Drives at magnitude of 40/100, with 0 rotation, for 5 seconds
