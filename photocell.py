#!/usr/bin/python  
import time  
import RPi.GPIO as GPIO

PHOTOCELL = 17

GPIO.setmode(GPIO.BCM)  
  
def RCtime():  
    reading = GPIO.input(PHOTOCELL)
    return reading  
  
while True:                                       
    print RCtime()  