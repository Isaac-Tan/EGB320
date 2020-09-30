import gpiozero
import time
import RPi.GPIO as GPIO
import numpy as np
#setup pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(24 , GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
# Left, Right
dir1 = [gpiozero.OutputDevice(18), gpiozero.OutputDevice(26)]
dir2 = [gpiozero.OutputDevice(23), gpiozero.OutputDevice(19)]
pwm = [GPIO.PWM(24, 100), GPIO.PWM(13, 100)]
#Init pwm at 0
pwm[0].start(0)
pwm[1].start(0)
m1mult = 1.0 #Left motor multiplier
m2mult = 0.9 #Right motor multiplier

def motor(mot, value):
  if (value > 0):
    dir1[mot].on()
    dir2[mot].off()
  elif (value < 0):
    dir1[mot].off()
    dir2[mot].on()
  pwm[mot].ChangeDutyCycle(value)

def drive(magnitude, rotation, tsec):
  motor(0, magnitude - rotation)
  motor(1, magnitude + rotation)
  time.sleep(tsec)
  pwm[0].stop()
  pwm[1].stop()

drive(40,0,5)

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
