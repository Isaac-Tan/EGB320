import RPi.GPIO as GPIO

RED = 16 #Red LED
YELLOW = 20 #Yellow LED
GREEN = 21 #Green LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN, GPIO.OUT) #Green LED
GPIO.setup(YELLOW, GPIO.OUT) #Yellow LED
GPIO.setup(RED, GPIO.OUT) #Red LED
GPIO.output(GREEN, GPIO.LOW)
GPIO.output(YELLOW, GPIO.LOW)
GPIO.output(RED, GPIO.LOW)

while (1):
    GPIO.output(GREEN, GPIO.HIGH)
    # GPIO.output(YELLOW, GPIO.HIGH)
    # GPIO.output(RED, GPIO.HIGH)