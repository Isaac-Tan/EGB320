import picamera
from time import *
from subprocess  import call 

with picamera.PiCamera() as camera:
    camera.start_recording("pivid.h264")
    sleep(30)
    camera.stop_recording()
    
command = "MP4Box -add pivid.h264 pivid.mp4"
call([command], shell=True)
print("vid conv")