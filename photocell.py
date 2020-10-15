#!/usr/bin/python  
import time  
import RPi.GPIO as GPIO

PHOTOCELL = 17

GPIO.setmode(GPIO.BCM)  
  
def RCtime():  
    reading = 0  
    GPIO.setup(PHOTOCELL, GPIO.OUT)  
    GPIO.output(PHOTOCELL, GPIO.LOW)
    time.sleep(0.1)  
    GPIO.setup(PHOTOCELL, GPIO.IN)  
    while (GPIO.input(PHOTOCELL) == GPIO.LOW):  
        reading += 1
    return reading  
  
while True:                                       
    print RCtime()  