# TO run this script, enter a frame path as cmd line arg: python3 color_thresholder_app.py frame01.png
import cv2
import numpy as np
import sys

if(len(sys.argv) < 2):
  exit(0)

def nothing(x):
  pass

####HSV Ranges####
##Sample
#Hue
s_hmin = 4
s_hmax = 40
#Saturation
s_smin = 225
s_smax = 255
#Value
s_vmin = 140
s_vmax = 255

##Rock
#Hue
r_hmin = 96
r_hmax = 110
#Saturation
r_smin = 71
r_smax = 255
#Value
r_vmin = 90
r_vmax = 255

##Obstacle
#Hue
o_hmin = 45
o_hmax = 56
#Saturation
o_smin = 133
o_smax = 255
#Value
o_vmin = 90
o_vmax = 255



#Video
cap = cv2.VideoCapture(sys.argv[1])
#Camera
#cap = cv2.VideoCapture(0)
cap.set(3, 320)									# Set the frame width
cap.set(4, 240)									# Set the frame height

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()): 
	# Capture frame-by-frame
	ret, frame = cap.read()
	if ret == True:
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

		s_min_arr = np.array([s_hmin,s_smin,s_vmin])
		s_max_arr = np.array([s_hmax,s_smax,s_vmax])

		r_min_arr = np.array([r_hmin,r_smin,r_vmin])
		r_max_arr = np.array([r_hmax,r_smax,r_vmax])

		o_min_ = np.array([o_hmin,o_smin,o_vmin])
		o_max_ = np.array([o_hmax,o_smax,o_vmax])

		s_mask = cv2.inRange(hsv, s_min_arr, s_max_arr)
		r_mask = cv2.inRange(hsv, r_min_arr, r_max_arr)
		o_mask = cv2.inRange(hsv, o_min_, o_max_)

		sample_img = cv2.bitwise_and(frame, frame, mask= s_mask)
		rock_img = cv2.bitwise_and(frame, frame, mask= r_mask)
		obstacle_img = cv2.bitwise_and(frame, frame, mask= o_mask)

		total_img = sample_img + rock_img + obstacle_img


		# cv2.imshow("Sample",sample_img)
		# cv2.imshow("Rock", rock_img)
		# cv2.imshow("Obstacle", obstacle_img)
		# blurred = cv2.GaussianBlur(total_img, (5, 5), 0)
		# thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

		cv2.namedWindow('Thresholder_App', cv2.WINDOW_NORMAL) #sets window as resizable
		cv2.imshow("Total", total_img)

		k = cv2.waitKey(1) & 0xFF

		# exit if q or esc are pressed
		if (k == ord('q') or k == 27):
			break

	else:
		break


# When everything done, release the video capture object
cap.release()
# Closes all the frames
cv2.destroyAllWindows()