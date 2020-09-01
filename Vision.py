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
r_vmin = 80
r_vmax = 255

##Obstacle
#Hue
o_hmin = 45
o_hmax = 56
#Saturation
o_smin = 133
o_smax = 255
#Value
o_vmin = 80
o_vmax = 255



#Video
cap = cv2.VideoCapture(sys.argv[1])
#Camera
#cap = cv2.VideoCapture(0)
# cap.set(3, 640)									# Set the frame width
# cap.set(4, 480)									# Set the frame height

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

		blurred = cv2.GaussianBlur(frame, (5, 5), 0)

		sample_img = cv2.bitwise_and(blurred, blurred, mask= s_mask)
		rock_img = cv2.bitwise_and(blurred, blurred, mask= r_mask)
		obstacle_img = cv2.bitwise_and(blurred, blurred, mask= o_mask)

		total_img = sample_img + rock_img + obstacle_img


		# cv2.imshow("Sample",sample_img)
		# cv2.imshow("Rock", rock_img)
		# cv2.imshow("Obstacle", obstacle_img)
		# rgb_tot = cv2.cvtColor(total_img, cv2.COLOR_HSV2RGB)
		# gray = cv2.cvtColor(rgb_tot, cv2.COLOR_BGR2GRAY)
		
		thresh = cv2.threshold(total_img, 0, 255, cv2.THRESH_BINARY)[1]
		kernel = np.ones((5,5),np.uint8)
		# opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
		# closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
		dilation = cv2.dilate(thresh,kernel,iterations = 1)
		erosion = cv2.erode(dilation,kernel,iterations = 2)
		opened = cv2.dilate(erosion,kernel,iterations = 1)
		blurred_thresh = cv2.GaussianBlur(opened, (5, 5), 0)

		cv2.namedWindow('Thresholder_App', cv2.WINDOW_NORMAL) #sets window as resizable
		cv2.imshow("Total", blurred_thresh)

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