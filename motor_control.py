#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  motor_control.py
#  
#  Copyright 2020  <pi@raspberrypi>
#  
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  
import gpiozero
import time
import RPi.GPIO as GPIO
#setup pins
GPIO.setmode(GPIO.BCM)
BackwardL = gpiozero.OutputDevice(18)
FowardL = gpiozero.OutputDevice(23)
GPIO.setup(24 , GPIO.OUT)
SpeedPWML = GPIO.PWM(24, 100)

BackwardR = gpiozero.OutputDevice(19)
FowardR = gpiozero.OutputDevice(26)
GPIO.setup(13, GPIO.OUT)
SpeedPWMR = GPIO.PWM(13, 100)

SpeedPWML.start(0)
SpeedPWMR.start(0)

#Isaac
# Left, Right
dir1 = []   #Forward
dir2 = []   #Backward
pwm = []

dir1(0) = gpiozero.OutputDevice(23)
dir1(1) = gpiozero.OutputDevice(26)

dir2(0) = gpiozero.OutputDevice(18)
dir2(1) = gpiozero.OutputDevice(19)

pwm(0) = GPIO.PWM(24, 100)
pwm(1) = GPIO.PWM(13, 100)


def motor(motor, value):
  if (value > 0):
    dir1(motor).on()
	  dir2(motor).off()
  elif (value < 0):
    dir1(motor).off()
	  dir2(motor).on()
  pwm(motor).ChangeDutyCycle(value)


def drive(magnitude, rotation, time):
  motor(0, magnitude - rotation)
  motor(1, magnitude - rotation)



def motor_control(speedR, speedL,  dirR, dirL, t):
  SpeedPWML.start(0)
  SpeedPWMR.start(0)
  if dirL == "foward":
 	BackwardL.on()
	FowardL.off()
  elif dirL == "back":
    BackwardL.off()
    FowardL.on()
  if dirR =="foward":
	FowardR.on()
	BackwardR.off()
  elif dirR == "back":
	BackwardR.on()
	FowardR.off()

  SpeedPWMR.ChangeDutyCycle(speedR)
  SpeedPWML.ChangeDutyCycle(speedL)
  time.sleep(t)
  SpeedPWML.stop()
  SpeedPWMR.stop()

def motor_control_r(speed, dir, t):
 SpeedPWMR.start(0)
 if dir =="foward":
    BackwardR.on()
    FowardR.off()
 elif dir == "back":
     BackwardR.off()
     FowardR.on()
 SpeedPWMR.ChangeDutyCycle(speed)
 time.sleep(t)
 SpeedPWMR.stop()
#motor_control_r(10, "foward", 4)
#motor_control(10,10, "foward", "foward",  6)
