import gpiozero
import time
import RPi.GPIO as GPIO
import numpy as np
#setup pins
GPIO.setmode(GPIO.BCM)
# BackwardL = gpiozero.OutputDevice(18)
# FowardL = gpiozero.OutputDevice(23)
# GPIO.setup(24 , GPIO.OUT)
# SpeedPWML = GPIO.PWM(24, 100)

# BackwardR = gpiozero.OutputDevice(19)
# FowardR = gpiozero.OutputDevice(26)
# GPIO.setup(13, GPIO.OUT)
# SpeedPWMR = GPIO.PWM(13, 100)

# SpeedPWML.start(0)
# SpeedPWMR.start(0)

#Isaac
GPIO.setup(24 , GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
# Left, Right
dir1 = [gpiozero.OutputDevice(23), gpiozero.OutputDevice(26)]
dir2 = [gpiozero.OutputDevice(18), gpiozero.OutputDevice(19)]
pwm = [GPIO.PWM(24, 100), GPIO.PWM(13, 100)]

# dir1[0] = gpiozero.OutputDevice(23)
# dir1[1] = gpiozero.OutputDevice(26)

# dir2[0] = gpiozero.OutputDevice(18)
# dir2[1] = gpiozero.OutputDevice(19)

# pwm[0] = GPIO.PWM(24, 100)
# pwm[1] = GPIO.PWM(13, 100)

def motor(motor, value):
  if (value > 0):
    dir1[motor].on()
    dir2[motor].off()
  elif (value < 0):
    dir1[motor].off()
    dir2[motor].on()
  pwm[motor].ChangeDutyCycle(value)

def drive(magnitude, rotation, time):
  motor(0, magnitude - rotation)
  motor(1, magnitude - rotation)
  time.sleep(t)
  pwm[0].stop()
  pwm[1].stop()

drive(1,0,10)

# def motor_control(speedR, speedL,  dirR, dirL, t):
#   SpeedPWML.start(0)
#   SpeedPWMR.start(0)
#   if (dirL == "foward"):
#  	  BackwardL.on()
# 	  FowardL.off()
#   elif (dirL == "back"):
#     BackwardL.off()
#     FowardL.on()
#   if (dirR =="foward"):
# 	  FowardR.on()
# 	  BackwardR.off()
#   elif (dirR == "back"):
# 	  BackwardR.on()
# 	  FowardR.off()
#   SpeedPWMR.ChangeDutyCycle(speedR)
#   SpeedPWML.ChangeDutyCycle(speedL)
#   time.sleep(t)
#   SpeedPWML.stop()
#   SpeedPWMR.stop()

# def motor_control_r(speed, dir, t):
#  SpeedPWMR.start(0)
#  if dir =="foward":
#     BackwardR.on()
#     FowardR.off()
#  elif dir == "back":
#      BackwardR.off()
#      FowardR.on()
#  SpeedPWMR.ChangeDutyCycle(speed)
#  time.sleep(t)
#  SpeedPWMR.stop()
#motor_control_r(10, "foward", 4)
#motor_control(10,10, "foward", "foward",  6)
