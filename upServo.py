import RPi.GPIO as GPIO
import time

servoPIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(4) # Initialization

p.ChangeDutyCycle(7)
time.sleep(0.5)
p.ChangeDutyCycle(4)
time.sleep(0.5)
p.stop()
GPIO.cleanup()
