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

cv2.namedWindow('Thresholder_App')

cv2.createTrackbar("VMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("VMin", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("SMax", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("SMin", "Thresholder_App",0,255,nothing)
cv2.createTrackbar("HMax", "Thresholder_App",0,179,nothing)
cv2.createTrackbar("HMin", "Thresholder_App",0,179,nothing)

cv2.setTrackbarPos("VMax", "Thresholder_App", 255)
cv2.setTrackbarPos("VMin", "Thresholder_App", 0)
cv2.setTrackbarPos("SMax", "Thresholder_App", 255)
cv2.setTrackbarPos("SMin", "Thresholder_App", 0)
cv2.setTrackbarPos("HMax", "Thresholder_App", 179)
cv2.setTrackbarPos("HMin", "Thresholder_App", 0)

def capture():
	#Video
	#cap = cv2.VideoCapture(sys.argv[1])
	#Camera
	cap = cv2.VideoCapture(0)
	cap.set(3, 320)									# Set the frame width
	cap.set(4, 240)									# Set the frame height
	# Check if camera opened successfully
	if (cap.isOpened() == False):
		print("Error opening video stream or file")

	while(cap.isOpened()):
	# Capture frame-by-frame
		ret, frame = cap.read()
		if ret == True:
			# height = frame.shape[0]
			# width = frame.shape[1]
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
	vmax=cv2.getTrackbarPos("VMax", "Thresholder_App")
	vmin=cv2.getTrackbarPos("VMin", "Thresholder_App")
	smax=cv2.getTrackbarPos("SMax", "Thresholder_App")
	smin=cv2.getTrackbarPos("SMin", "Thresholder_App")
	hmax=cv2.getTrackbarPos("HMax", "Thresholder_App")
	hmin=cv2.getTrackbarPos("HMin", "Thresholder_App")
	min_ = np.array([hmin,smin,vmin])
	max_ = np.array([hmax,smax,vmax])
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, min_, max_)
	thresholded_img = cv2.bitwise_and(frame, frame, mask = mask)
	cv2.imshow("Thresholder_App", thresholded_img)

	k = cv2.waitKey(1) & 0xFF
	# exit if r is pressed
	if (k == ord('s')):
		f = open("sample.txt", "w")
		f.write(str(hmin) + "\n")
		f.write(str(hmax) + "\n")
		f.write(str(smin) + "\n")
		f.write(str(smax) + "\n")
		f.write(str(vmin) + "\n")
		f.write(str(vmax))
		f.close()
	if (k == ord('r')):
		f = open("rock.txt", "w")
		f.write(str(hmin) + "\n")
		f.write(str(hmax) + "\n")
		f.write(str(smin) + "\n")
		f.write(str(smax) + "\n")
		f.write(str(vmin) + "\n")
		f.write(str(vmax))
		f.close()
	if (k == ord('o')):
		f = open("obstacle.txt", "w")
		f.write(str(hmin) + "\n")
		f.write(str(hmax) + "\n")
		f.write(str(smin) + "\n")
		f.write(str(smax) + "\n")
		f.write(str(vmin) + "\n")
		f.write(str(vmax))
		f.close()
    
def cleanUp():
	# Closes all the frames
	cv2.destroyAllWindows()

if __name__ == '__main__':
	now = time.time()
	capture()
	elapsed = time.time() - now