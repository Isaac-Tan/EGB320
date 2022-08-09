# TO run this script, enter a frame path as cmd line arg: python3 color_thresholder_app.py frame01.png
import cv2
import numpy as np
import sys
import time

# if(len(sys.argv) < 2):
# 	exit(0)

def nothing(x):
	pass

cap = None

writing = False

obj = 'Live_feed'

cv2.namedWindow('Thresholder_App', cv2.WINDOW_NORMAL)
#cv2.namedWindow("Live_feed", cv2.WINDOW_NORMAL)

cv2.createTrackbar("HMax", "Thresholder_App",0,179,nothing)
cv2.createTrackbar("HMin", "Thresholder_App",0,179,nothing)
cv2.createTrackbar("SMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("SMin", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("VMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("VMin", "Thresholder_App",0,255,nothing)

hmax = 179
hmin = 0
smax = 255
smin = 0
vmax = 255
vmin = 0

cv2.setTrackbarPos("HMax", "Thresholder_App", hmax)
cv2.setTrackbarPos("HMin", "Thresholder_App", hmin)
cv2.setTrackbarPos("SMax", "Thresholder_App", smax)
cv2.setTrackbarPos("SMin", "Thresholder_App", smin)
cv2.setTrackbarPos("VMax", "Thresholder_App", vmax)
cv2.setTrackbarPos("VMin", "Thresholder_App", vmin)

def capture():
	#Video
	#cap = cv2.VideoCapture(sys.argv[1])
	#Camera
	cap = cv2.VideoCapture(0)
	cap.set(3, 320)									# Set the frame width
	cap.set(4, 240)									# Set the frame height
	cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
	cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
	cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
	# Check if camera opened successfully
	if (cap.isOpened() == False):
		print("Error opening video stream or file")

	while(cap.isOpened()):
	# Capture frame-by-frame
		ret, frame = cap.read()
		if ret == True:
			# height = frame.shape[0]
			# width = frame.shape[1]
			frame = cv2.rotate(frame, cv2.ROTATE_180)
			cv2.normalize(frame, frame, 0, 255, cv2.NORM_MINMAX)
			process(frame)
			k = cv2.waitKey(1) & 0xFF

			# exit if q or esc are pressed
			if (k == ord('q') or k == 27):
				break

		else:
			break
    # When everything done, release the video capture object
	cap.release()
	cleanUp()

def process(frame):
	global hmax, hmin, smax, smin, vmax, vmin
	hmax=cv2.getTrackbarPos("HMax", "Thresholder_App")
	hmin=cv2.getTrackbarPos("HMin", "Thresholder_App")
	smax=cv2.getTrackbarPos("SMax", "Thresholder_App")
	smin=cv2.getTrackbarPos("SMin", "Thresholder_App")
	vmax=cv2.getTrackbarPos("VMax", "Thresholder_App")
	vmin=cv2.getTrackbarPos("VMin", "Thresholder_App")

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	global obj
	if (obj == 'Sample'):
		min_1 = np.array([0,smin,vmin])
		max_1 = np.array([hmin,smax,vmax])
		min_2 = np.array([hmax,smin,vmin])
		max_2 = np.array([179,smax,vmax])

		mask1 = cv2.inRange(hsv, min_1, max_1)
		mask2 = cv2.inRange(hsv, min_2, max_2)
		mask = mask1 + mask2

	else:
		min_1 = np.array([hmin,smin,vmin])
		max_1 = np.array([hmax,smax,vmax])
		mask = cv2.inRange(hsv, min_1, max_1)
	thresholded_img = cv2.bitwise_and(frame, frame, mask = mask)
	global writing
	if (writing == False):
		cv2.putText(thresholded_img,"Reading", (15, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	elif (writing == True):
		cv2.putText(thresholded_img,"Writing", (15, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	cv2.putText(thresholded_img, obj, (270, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	cv2.imshow('Thresholder_App', thresholded_img)

	k = cv2.waitKey(1) & 0xFF
	# exit if r is pressed
	if (k == ord('w')):
		if (writing == False):
			writing = True
		elif (writing == True):
			writing = False

	if (writing == True):
		if (k == ord('s')):
			obj = "Sample"
			f = open("sample.txt", "w")
			f.write(str(hmin) + "\n")
			f.write(str(hmax) + "\n")
			f.write(str(smin) + "\n")
			f.write(str(smax) + "\n")
			f.write(str(vmin) + "\n")
			f.write(str(vmax))
			f.close()
		if (k == ord('r')):
			obj = "Rock"
			f = open("rock.txt", "w")
			f.write(str(hmin) + "\n")
			f.write(str(hmax) + "\n")
			f.write(str(smin) + "\n")
			f.write(str(smax) + "\n")
			f.write(str(vmin) + "\n")
			f.write(str(vmax))
			f.close()
		if (k == ord('o')):
			obj = "Obstacle"
			f = open("obstacle.txt", "w")
			f.write(str(hmin) + "\n")
			f.write(str(hmax) + "\n")
			f.write(str(smin) + "\n")
			f.write(str(smax) + "\n")
			f.write(str(vmin) + "\n")
			f.write(str(vmax))
			f.close()
		if (k == ord('l')):
			obj = "Lander"
			f = open("lander.txt", "w")
			f.write(str(hmin) + "\n")
			f.write(str(hmax) + "\n")
			f.write(str(smin) + "\n")
			f.write(str(smax) + "\n")
			f.write(str(vmin) + "\n")
			f.write(str(vmax))
			f.close()

	if (writing == False):
		if (k == ord('s')):
			obj = "Sample"
			f = open("sample.txt", "r")
			hmin = (int(f.readline()))
			hmax = (int(f.readline()))
			smin = (int(f.readline()))
			smax = (int(f.readline()))
			vmin = (int(f.readline()))
			vmax = (int(f.readline()))
			f.close()
		if (k == ord('r')):
			obj = "Rock"
			f = open("rock.txt", "r")
			hmin = (int(f.readline()))
			hmax = (int(f.readline()))
			smin = (int(f.readline()))
			smax = (int(f.readline()))
			vmin = (int(f.readline()))
			vmax = (int(f.readline()))
			f.close()
		if (k == ord('o')):
			obj = "Obstacle"
			f = open("obstacle.txt", "r")
			hmin = (int(f.readline()))
			hmax = (int(f.readline()))
			smin = (int(f.readline()))
			smax = (int(f.readline()))
			vmin = (int(f.readline()))
			vmax = (int(f.readline()))
			f.close()
		if (k == ord('l')):
			obj = "Lander"
			f = open("lander.txt", "r")
			hmin = (int(f.readline()))
			hmax = (int(f.readline()))
			smin = (int(f.readline()))
			smax = (int(f.readline()))
			vmin = (int(f.readline()))
			vmax = (int(f.readline()))
			f.close()
	time.sleep(0.0000001)
	setVals()

def setVals():
	#Sets Trackbar vals
	cv2.setTrackbarPos("HMax", "Thresholder_App", hmax)
	cv2.setTrackbarPos("HMin", "Thresholder_App", hmin)
	cv2.setTrackbarPos("SMax", "Thresholder_App", smax)
	cv2.setTrackbarPos("SMin", "Thresholder_App", smin)
	cv2.setTrackbarPos("VMax", "Thresholder_App", vmax)
	cv2.setTrackbarPos("VMin", "Thresholder_App", vmin)

def cleanUp():
	# Closes all the frames
	cv2.destroyAllWindows()

if __name__ == '__main__':
	now = time.time()
	capture()
	elapsed = time.time() - now
