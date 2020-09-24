# TO run this script, enter a frame path as cmd line arg: python3 color_thresholder_app.py frame01.png
import cv2
import numpy as np
import sys
import argparse
import imutils
import time

FREQUENCY = 50 #Hz
INTERVAL = 1.0/FREQUENCY
FOCAL_LEN = 3.04 #mm
SENSOR_HEIGHT = 2.76 #mm
SENSOR_WIDTH = 3.68 #mm
OBST_HEIGHT = 150 #mm
ROCK_HEIGHT = 70 #mm
SAMPLE_HEIGHT = 40 #mm
HEIGHT = 240 #screen height
WIDTH = 320 #screen width

#HSV Value arrays
s_min_1 = [int]*3
s_max_1 = [int]*3
s_min_2 = [int]*3
s_max_2 = [int]*3
r_min_ = [int]*3
r_max_ = [int]*3
o_min_ = [int]*3
o_max_ = [int]*3

s_max_arr1 = []
s_min_arr1 = []
s_max_arr2 = []
s_min_arr2 = []
r_max_arr = []
r_min_arr = []
o_max_arr = []
o_min_arr = []

Sample_list = []
Rock_list = []
Obstacle_list = []

init = False

#Video
#cap = cv2.VideoCapture(sys.argv[1])
#Camera
cap = cv2.VideoCapture(-1)
cap.set(3, 320)									# Set the frame WIDTH
cap.set(4, 240)									# Set the frame HEIGHT

class Sample:
	#'Class for Samples'
	sampleCount = 0
	def __init__(self, ID, Dist, Bearing, cX, cY):
		self.Dist = Dist
		self.Bearing = Bearing
		self.ID = ID
		self.cX = cX
		self.cY = cY
		Sample.sampleCount += 1

	def __del__(self):
		Sample.sampleCount -= 1

class Rock:
	#'Class for Rocks'
	rockCount = 0
	def __init__(self, ID, Dist, Bearing, cX, cY):
		self.Dist = Dist
		self.Bearing = Bearing
		self.ID = ID
		self.cX = cX
		self.cY = cY
		Rock.rockCount += 1
	def __del__(self):
		Rock.rockCount -= 1

class Obstacle:
	#'Class for Obstacles'
	obstacleCount = 0
	def __init__(self, ID, Dist, Bearing, cX, cY):
		self.Dist = Dist
		self.Bearing = Bearing
		self.ID = ID
		self.cX = cX
		self.cY = cY
		Obstacle.obstacleCount += 1
	def __del__(self):
		Obstacle.obstacleCount -= 1

def bounds():
	##Gets the HSV values from the .txt files
	#Sample
	f = open("sample.txt","r")
	for i in range(3):
		s_min_1[i] = (int(f.readline()))
		s_max_1[i] = (int(f.readline()))
	f.seek(0)
	for i in range(3):
		s_min_2[i] = (int(f.readline()))
		s_max_2[i] = (int(f.readline()))
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

	global s_min_arr1
	global s_max_arr1
	s_max_1[0] = s_min_1[0]
	s_min_1[0] = 0
	s_min_arr1 = np.array(s_min_1)
	s_max_arr1 = np.array(s_max_1)
	global s_min_arr2
	global s_max_arr2
	s_min_2[0] = s_max_2[0]
	s_max_2[0] = 179
	s_min_arr2 = np.array(s_min_2)
	s_max_arr2 = np.array(s_max_2)

	global r_min_arr
	r_min_arr = np.array(r_min_)
	global r_max_arr
	r_max_arr = np.array(r_max_)

	global o_min_arr
	o_min_arr = np.array(o_min_)
	global o_max_arr
	o_max_arr = np.array(o_max_)


def thresh(input_frame, type, total_img):
	gray = input_frame[:, :, 2]
	thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
	kernel = np.ones((5,5),np.uint8)
	dilation = cv2.dilate(thresh,kernel,iterations = 2)
	erosion = cv2.erode(dilation,kernel,iterations = 4)
	opened = cv2.dilate(erosion,kernel,iterations = 2)
	blurred_thresh = cv2.GaussianBlur(opened, (5, 5), 0)
	ims = blurred_thresh
	cnts = cv2.findContours(ims, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	i = 0
	# loop over the contours
	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])#Centre x-coord
		cY = int(M["m01"] / M["m00"])#Centre y-coord
		# compute bearing of the contour
		bearing = round(31.1 * ((cX - (WIDTH/2.0))/(WIDTH/2.0)),3)
		# get height/width of contour
		x,y,h,w = cv2.boundingRect(c)
		#calculate distance
		if (type == 0):
			dist = round(0.1*(FOCAL_LEN*SAMPLE_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (255, 69, 0), 2)
		elif (type == 1):
			dist = round(0.1*(FOCAL_LEN*ROCK_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (0, 0, 255), 2)
		elif (type == 2):
			dist = round(0.1*(FOCAL_LEN*OBST_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (0, 255, 0), 2)
		cv2.circle(total_img, (cX, cY), 7, (255, 0, 0), -1)
		cv2.putText(total_img, "R: " + str(dist) + "cm", (cX - 15, cY + 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "B: " + str(bearing), (cX - 15, cY + 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		if (type == 0):
			global Sample_list
			sample = Sample(i,dist,bearing,cX,cY)
			Sample_list.append(Sample(i,dist,bearing,cX,cY))
			cv2.putText(total_img, "Sample", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		elif (type == 1):
			global Rock_list
			rock = Rock(i,dist,bearing,cX,cY)
			Rock_list.append(Rock(i,dist,bearing,cX,cY))
			cv2.putText(total_img, "Rock", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		elif (type == 2):
			global Obstacle_list
			obstacle = Obstacle(i,dist,bearing,cX,cY)
			Obstacle_list.append(Obstacle(i,dist,bearing,cX,cY))
			del obstacle
			cv2.putText(total_img, "Obstacle", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		# elif (type == 3):
		# 	Lander[c] = Lander(c,dist,bearing,cX,cY)
		i = i + 1
	return total_img


def capture():
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
	now = time.time()
	frame = cv2.rotate(frame, cv2.ROTATE_180)
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	s_mask1 = cv2.inRange(hsv, s_min_arr1, s_max_arr1)
	s_mask2 = cv2.inRange(hsv, s_min_arr2, s_max_arr2)
	s_mask = s_mask1 + s_mask2
	#Note S H range is 0-179 excluding 50-150
	r_mask = cv2.inRange(hsv, r_min_arr, r_max_arr)
	o_mask = cv2.inRange(hsv, o_min_arr, o_max_arr)

	blurred = cv2.GaussianBlur(frame, (5, 5), 0)

	# sample_img1 = cv2.bitwise_and(blurred, blurred, mask= s_mask1)
	# sample_img2 = cv2.bitwise_and(blurred, blurred, mask= s_mask2)
	# sample_img = sample_img1 + sample_img2
	sample_img = cv2.bitwise_and(blurred, blurred, mask= s_mask)
	rock_img = cv2.bitwise_and(blurred, blurred, mask= r_mask)
	obstacle_img = cv2.bitwise_and(blurred, blurred, mask= o_mask)

	# total_img = sample_img + rock_img + obstacle_img
	total_img = frame


	#cv2.imshow("Sample",sample_img)
	# cv2.imshow("Rock", rock_img)
	# cv2.imshow("Obstacle", obstacle_img)
	Sample_list = []
	Rock_list = []
	Obstacle_list = []
	sample = thresh(sample_img, 0, total_img)
	rock = thresh(rock_img, 1,total_img)
	obstacle = thresh(obstacle_img, 2,total_img)

	# draw a line down the centre of the screen
	cv2.line(total_img, ((int(WIDTH/2)),0), ((int(WIDTH/2)),int(HEIGHT)), (255, 255, 255))
	

	elapsed = time.time() - now
	rate = round(1.0/elapsed,0)
	# if (rate > FREQUENCY):
	# 	time.sleep(INTERVAL - elapsed)
	elapsed2 = time.time() - now
	rate2 = round(1.0/elapsed2,0)
	cv2.putText(total_img, "Frquency: " + str(rate2) + "Hz", (15, 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
	cv2.imshow("Total", total_img)


def cleanUp():
	# Closes all the frames
	cv2.destroyAllWindows()

def main():
	capture()

if __name__ == '__main__':
	if (init == False):
		bounds()
		init = True
	main()
