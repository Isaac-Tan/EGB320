import RPi.GPIO as GPIO

RED = 16 #Red LED
YELLOW = 20 #Yellow LED
GREEN = 21 #Green LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN, GPIO.OUT) #Green LED
GPIO.setup(YELLOW, GPIO.OUT) #Yellow LED
GPIO.setup(RED, GPIO.OUT) #Red LED

while (1):
    GPIO.output(GREEN, GPIO.HIGH)
    # GPIO.output(YELLOW, GPIO.HIGH)
    # GPIO.output(RED, GPIO.HIGH)