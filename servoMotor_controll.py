import RPi.GPIO as GPIO
import time

def servoControl(duty, ti):
	servoPIN = 17
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(servoPIN, GPIO.OUT)

	p = GPIO.PWM(servoPIN, 50)
	p.start(2.5)
	t = 0
	try:
    	  while t==0:
	 	p.ChangeDutyCycle(duty)
	 	time.sleep(ti)
		t = t + 1
	except KeyboardInterrupt:
 	 p.stop()
 	 GPIO.cleanup()
servoControl(1, 0.14)

