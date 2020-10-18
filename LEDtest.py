import RPi.GPIO as GPIO

RED = 21 #Red LED
YELLOW = 20 #Yellow LED
GREEN = 16 #Green LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(GREEN, GPIO.OUT) #Green LED
GPIO.setup(YELLOW, GPIO.OUT) #Yellow LED
GPIO.setup(RED, GPIO.OUT) #Red LED

counter = 0

try: 
    while (counter < 1000000):
        GPIO.output(GREEN, GPIO.HIGH)
        # GPIO.output(YELLOW, GPIO.HIGH)
        # GPIO.output(RED, GPIO.HIGH)
        counter += 1
except:
    print("/n", counter)
finally:
    GPIO.cleanup() # this ensures a clean exit  
    