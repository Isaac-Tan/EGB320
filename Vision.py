# TO run this script, enter a frame path as cmd line arg: python3 color_thresholder_app.py frame01.png
import cv2
import numpy as np
import sys
import argparse
import imutils
import time

FREQUENCY = 10 #Hz
INTERVAL = 1.0/FREQUENCY

#HSV Value arrays
s_min_ = [int]*3
s_max_ = [int]*3
r_min_ = [int]*3
r_max_ = [int]*3
o_min_ = [int]*3
o_max_ = [int]*3

init = False

def bounds():
	##Gets the HSV values from the .txt files
	#Sample
	f = open("sample.txt","r")
	for i in range(3):
		s_min_[i] = (int(f.readline()))
		s_max_[i] = (int(f.readline()))
	f.close()
	#Rock
	f = open("rock.txt","r")
	for i in range(3):
		r_min_[i] = (int(f.readline()))
		r_max_[i] = (int(f.readline()))
	f.close()
	#Obstacle
	f = open("obstacle.txt","r")
	for i in range(3):
		o_min_[i] = (int(f.readline()))
		o_max_[i] = (int(f.readline()))
	f.close()


# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", required=True, help="path to the input image")
# args = vars(ap.parse_args())
cap = None
HEIGHT = 240
WIDTH = 320

def capture():
	#Video
	cap = cv2.VideoCapture(sys.argv[1])
	#Camera
	#cap = cv2.VideoCapture(0)
	cap.set(3, 320)									# Set the frame WIDTH
	cap.set(4, 240)									# Set the frame HEIGHT
	# Check if camera opened successfully
	if (cap.isOpened()== False): 
	  print("Error opening video stream or file")

	while(cap.isOpened()): 
	# Capture frame-by-frame
		ret, frame = cap.read()
		if ret == True:
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
	frame = cv2.rotate(frame, cv2.ROTATE_180)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	s_min_arr = np.array(s_min_)
	s_max_arr = np.array(s_max_)

	r_min_arr = np.array(r_min_)
	r_max_arr = np.array(r_max_)

	o_min_arr = np.array(o_min_)
	o_max_arr = np.array(o_max_)

	s_mask = cv2.inRange(hsv, s_min_arr, s_max_arr)
	r_mask = cv2.inRange(hsv, r_min_arr, r_max_arr)
	o_mask = cv2.inRange(hsv, o_min_arr, o_max_arr)

	blurred = cv2.GaussianBlur(frame, (5, 5), 0)

	sample_img = cv2.bitwise_and(blurred, blurred, mask= s_mask)
	rock_img = cv2.bitwise_and(blurred, blurred, mask= r_mask)
	obstacle_img = cv2.bitwise_and(blurred, blurred, mask= o_mask)

	total_img = sample_img + rock_img + obstacle_img


	# cv2.imshow("Sample",sample_img)
	# cv2.imshow("Rock", rock_img)
	# cv2.imshow("Obstacle", obstacle_img)
	rgb_s = cv2.cvtColor(sample_img, cv2.COLOR_HSV2RGB)
	rgb_r = cv2.cvtColor(rock_img, cv2.COLOR_HSV2RGB)
	rgb_o = cv2.cvtColor(obstacle_img, cv2.COLOR_HSV2RGB)

	gray_s = cv2.cvtColor(rgb_s, cv2.COLOR_BGR2GRAY)
	gray_r = cv2.cvtColor(rgb_r, cv2.COLOR_BGR2GRAY)
	gray_o = cv2.cvtColor(rgb_o, cv2.COLOR_BGR2GRAY)
	
	thresh_s = cv2.threshold(gray_s, 0, 255, cv2.THRESH_BINARY)[1]
	thresh_r = cv2.threshold(gray_r, 0, 255, cv2.THRESH_BINARY)[1]
	thresh_o = cv2.threshold(gray_o, 0, 255, cv2.THRESH_BINARY)[1]
	kernel = np.ones((5,5),np.uint8)
	# opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
	# closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
	dilation_s = cv2.dilate(thresh_s,kernel,iterations = 15)
	dilation_r = cv2.dilate(thresh_r,kernel,iterations = 15)
	dilation_o = cv2.dilate(thresh_o,kernel,iterations = 15)

	erosion_s = cv2.erode(dilation_s,kernel,iterations = 30)
	erosion_r = cv2.erode(dilation_r,kernel,iterations = 30)
	erosion_o = cv2.erode(dilation_o,kernel,iterations = 30)

	opened_s = cv2.dilate(erosion_s,kernel,iterations = 15)
	opened_r = cv2.dilate(erosion_r,kernel,iterations = 15)
	opened_o = cv2.dilate(erosion_o,kernel,iterations = 15)

	blurred_thresh_s = cv2.GaussianBlur(opened_s, (5, 5), 0)
	blurred_thresh_r = cv2.GaussianBlur(opened_r, (5, 5), 0)
	blurred_thresh_o = cv2.GaussianBlur(opened_o, (5, 5), 0)

	ims_s = blurred_thresh_s.copy()
	ims_r = blurred_thresh_r.copy()
	ims_o = blurred_thresh_o.copy()

	# find contours in the thresholded image
	cnts_s = cv2.findContours(ims_s, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_s = imutils.grab_contours(cnts_s)

	# loop over the contours
	for c in cnts_s:
		# compute the center of the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])#Centre x-coord
		cY = int(M["m01"] / M["m00"])#Centre y-coord
		# compute bearing of the contour
		bearing = round(31.1 * ((cX - (WIDTH/2))/(WIDTH/2)),3)
		# draw the contour and center of the shape on the image
		cv2.drawContours(total_img, [c], -1, (0, 255, 0), 2)
		cv2.circle(total_img, (cX, cY), 7, (255, 0, 0), -1)
		# get height/width of contour
		x,y,h,w = cv2.boundingRect(c)
		#calculate distance
		dist = round((3.04*700)/(h),3)
		cv2.putText(total_img, "R: " + str(dist), (cX - 15, cY + 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "B: " + str(bearing), (cX - 15, cY + 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "Sample", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		# show the image
		# cv2.imshow("Image", total_img)
		#cv2.waitKey(0)
		# find contours in the thresholded image
	cnts_r = cv2.findContours(ims_r, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_r = imutils.grab_contours(cnts_r)

	# loop over the contours
	for c in cnts_r:
		# compute the center of the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])#Centre x-coord
		cY = int(M["m01"] / M["m00"])#Centre y-coord
		# compute bearing of the contour
		bearing = round(31.1 * ((cX - (WIDTH/2))/(WIDTH/2)),3)
		# draw the contour and center of the shape on the image
		cv2.drawContours(total_img, [c], -1, (0, 255, 0), 2)
		cv2.circle(total_img, (cX, cY), 7, (255, 0, 0), -1)
		# get height/width of contour
		x,y,h,w = cv2.boundingRect(c)
		#calculate distance
		dist = round((3.04*700)/(h),3)
		cv2.putText(total_img, "R: " + str(dist), (cX - 15, cY + 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "B: " + str(bearing), (cX - 15, cY + 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "Rock", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	
		# find contours in the thresholded image
	cnts_o = cv2.findContours(ims_o, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts_o = imutils.grab_contours(cnts_o)

	# loop over the contours
	for c in cnts_o:
		# compute the center of the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])#Centre x-coord
		cY = int(M["m01"] / M["m00"])#Centre y-coord
		# compute bearing of the contour
		bearing = round(31.1 * ((cX - (WIDTH/2))/(WIDTH/2)),3)
		# draw the contour and center of the shape on the image
		cv2.drawContours(total_img, [c], -1, (0, 255, 0), 2)
		cv2.circle(total_img, (cX, cY), 7, (255, 0, 0), -1)
		# get height/width of contour
		x,y,h,w = cv2.boundingRect(c)
		#calculate distance
		dist = round((3.04*700)/(h),3)
		cv2.putText(total_img, "R: " + str(dist), (cX - 15, cY + 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "B: " + str(bearing), (cX - 15, cY + 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "Obstacle", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	#cv2.namedWindow('Thresholder_App', cv2.WINDOW_NORMAL) #sets window as resizable
			# draw a line down the centre of the screen
	cv2.line(total_img, ((int(WIDTH/2)),0), ((int(WIDTH/2)),int(HEIGHT)), (255, 255, 255))
	cv2.imshow("Total", total_img)
	# time.sleep(0.0001)


def cleanUp():
	# Closes all the frames
	cv2.destroyAllWindows()

if __name__ == '__main__':
	now = time.time()
	if (init == False):
		bounds()
		init = True
	capture()
	elapsed = time.time() - now
	# time.sleep(INTERVAL - elapsed)