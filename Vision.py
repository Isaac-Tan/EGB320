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
		self.Dist = Dist	#Distance from camera to object
		self.Bearing = Bearing		#Angle from centre of POV to object
		self.ID = ID	#ID of object
		self.cX = cX	#x-coord of the centre of the object
		self.cY = cY	#y-coord of the centre of the object
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
	#min hue mask 1 from 0 to min of txt file
	s_max_1[0] = s_min_1[0]
	s_min_1[0] = 0
	s_min_arr1 = np.array(s_min_1)
	s_max_arr1 = np.array(s_max_1)
	global s_min_arr2
	global s_max_arr2
	#min hue mask 2 from min of txt file to 179
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
	#input frame, type (sample, rock, obst, etc), output frame
	gray = input_frame[:, :, 2]		#sets to the 3rd channel of input (greyscale)
	thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]		#converts greyscale to binary
	kernel = np.ones((5,5),np.uint8)	#creates a 5x5 matrix of ones for dilation and erotion
	dilation = cv2.dilate(thresh,kernel,iterations = 2)		#dilates anything larger than the 5x5 matrix, twice
	erosion = cv2.erode(dilation,kernel,iterations = 4)		#erodes anything larger than the 5x5 matrix, 4 times
	opened = cv2.dilate(erosion,kernel,iterations = 2)		#dilates anything larger than the 5x5 matrix, twice
	blurred_thresh = cv2.GaussianBlur(opened, (5, 5), 0)	#applies gausian blur of 5x5
	ims = blurred_thresh	#somewhat redundant but smaller variable name
	cnts = cv2.findContours(ims, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)		#finds the contours and stores them in cnts
	cnts = imutils.grab_contours(cnts)		#grabs contours from cnts
	i = 0	#used for library ID
	# loop over the contours
	for c in cnts:
		# compute the center of the contour
		M = cv2.moments(c)	#Moments
		cX = int(M["m10"] / M["m00"])#Centre x-coord
		cY = int(M["m01"] / M["m00"])#Centre y-coord
		# compute bearing of the contour
		bearing = round(31.1 * ((cX - (WIDTH/2.0))/(WIDTH/2.0)),3)
		# get height/width of contour
		x,y,h,w = cv2.boundingRect(c)
		#calculate distance
		if (type == 0):
			#dist(cm) = 0.1 x (focal length(mm) x real sample height(mm) x screen height(px))/(pixel height(px) x sensor height(mm))
			dist = round(0.1*(FOCAL_LEN*SAMPLE_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (0, 69, 255), 2)	#Draws bounding box on output img around contour #c
		elif (type == 1):
			dist = round(0.1*(FOCAL_LEN*ROCK_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (255, 0, 0), 2)
		elif (type == 2):
			dist = round(0.1*(FOCAL_LEN*OBST_HEIGHT*HEIGHT)/(h*SENSOR_HEIGHT),3)
			cv2.drawContours(total_img, [c], -1, (0, 255, 0), 2)

		cv2.circle(total_img, (cX, cY), 7, (255, 0, 0), -1)		#draws a circle at the centre of the contour
		#Displays range and bearing on output img
		cv2.putText(total_img, "R: " + str(dist) + "cm", (cX - 15, cY + 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		cv2.putText(total_img, "B: " + str(bearing), (cX - 15, cY + 30),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

		if (type == 0):		#if sample
			global Sample_list		#Global needs to be called to store into a global variable
			sample = Sample(i,dist,bearing,cX,cY)
			Sample_list.append(Sample(i,dist,bearing,cX,cY))	#adds this sample to the class of samples
			#Displays "sample" in the centre of the contour
			cv2.putText(total_img, "Sample", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		elif (type == 1):	#if rock
			global Rock_list
			rock = Rock(i,dist,bearing,cX,cY)
			Rock_list.append(Rock(i,dist,bearing,cX,cY))
			cv2.putText(total_img, "Rock", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		elif (type == 2):	#if obstacle
			global Obstacle_list
			obstacle = Obstacle(i,dist,bearing,cX,cY)
			Obstacle_list.append(Obstacle(i,dist,bearing,cX,cY))
			del obstacle
			cv2.putText(total_img, "Obstacle", (cX - 15, cY - 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
		# elif (type == 3):
		# 	Lander[c] = Lander(c,dist,bearing,cX,cY)
		i = i + 1	#add 1 to the ID of object class

	return total_img		#return output image


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
	now = time.time()	#start process time
	frame = cv2.rotate(frame, cv2.ROTATE_180)		#rotate the frame 180'
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)	#convert rgb to hsv

	#sampel has 2 mask because hue wraps from 179 around to 0
	s_mask1 = cv2.inRange(hsv, s_min_arr1, s_max_arr1)		#sample mask 1
	s_mask2 = cv2.inRange(hsv, s_min_arr2, s_max_arr2)		#sample mask 2
	s_mask = s_mask1 + s_mask2		#adds sample masks together
	
	r_mask = cv2.inRange(hsv, r_min_arr, r_max_arr)		#rock mask
	o_mask = cv2.inRange(hsv, o_min_arr, o_max_arr)		#obstacle mask

	blurred = cv2.GaussianBlur(frame, (5, 5), 0)		#blur the frame with a 5x5

	# sample_img1 = cv2.bitwise_and(blurred, blurred, mask= s_mask1)
	# sample_img2 = cv2.bitwise_and(blurred, blurred, mask= s_mask2)
	# sample_img = sample_img1 + sample_img2

	#overlay the mask on the blurred image for bitwise and
	sample_img = cv2.bitwise_and(blurred, blurred, mask= s_mask)
	rock_img = cv2.bitwise_and(blurred, blurred, mask= r_mask)
	obstacle_img = cv2.bitwise_and(blurred, blurred, mask= o_mask)

	# total_img = sample_img + rock_img + obstacle_img
	total_img = frame


	#cv2.imshow("Sample",sample_img)
	# cv2.imshow("Rock", rock_img)
	# cv2.imshow("Obstacle", obstacle_img)

	#clear the current classes
	Sample_list = []
	Rock_list = []
	Obstacle_list = []

	#object frame = thresh(input img, obj type, output img)
	sample = thresh(sample_img, 0, total_img)
	rock = thresh(rock_img, 1,total_img)
	obstacle = thresh(obstacle_img, 2,total_img)

	# draw a line down the centre of the screen
	cv2.line(total_img, ((int(WIDTH/2)),0), ((int(WIDTH/2)),int(HEIGHT)), (255, 255, 255))
	
	elapsed = time.time() - now			#end process time
	rate = round(1.0/elapsed,0)			#process rate
	if (rate > FREQUENCY):				#only sleep if process rate is faster than desired freq
		time.sleep(INTERVAL - elapsed)
	elapsed2 = time.time() - now
	rate2 = round(1.0/elapsed2,0)
	#Display Frequency in top left corner
	cv2.putText(total_img, "Frequency: " + str(rate2) + "Hz", (15, 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

	cv2.imshow("Total", total_img)		#display final output img


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
