#!/usr/bin/python  
import time  
import RPi.GPIO as GPIO

PHOTOCELL = 17

GPIO.setmode(GPIO.BCM)  
avg = []
thresh = 0
scale = 0.2
  
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
    if (len(avg) < 30):
        avg.append(RCtime())
    else:
        thresh = float(sum(avg) / len(avg))
        tolerance = scale * thresh
        thresh = thresh + tolerance
    print("Thresh: ", thresh)
    print(RCtime())