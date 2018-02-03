# USAGE
# python bright.py --image images/retina.png --radius 41 --thresh 50
# python bright.py --image images/retina-noise.png --radius 41 --thresh 50
# python bright.py --image images/moon.png --radius 61 --thresh 50
# python bright.py --image ../LedDetector/data/output/600.jpg --radius 41 --thresh 60

# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2


# -- Comment in during runtime --
#import MatchSpecificData.h

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", help = "path to the image file")
ap.add_argument("-r", "--radius", type = int,
	help = "radius of Gaussian blur; must be odd")
ap.add_argument("-t", "--thresh", type = int)
args = vars(ap.parse_args())

# load the image and convert it to grayscale
image = cv2.imread(args["image"])
if image != None:
    orig = image.copy()
    detect = image.copy()
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#Check what color alliance we are
alliance_color = 1


#If our alliance color is blue, the robot will only detect blue.
#If our alliance color is red, the robot will detect both leds, but

#Filter out anything that's not blue
if alliance_color == 0:
	hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([110,50,50])
        upper_red = np.array([130,255,255])
 
	# Here we are defining range of bluecolor in HSV
	# This creates a mask of blue coloured 
	# objects found in the frame.
	mask = cv2.inRange(hsv, lower_red, upper_red)
	 
	# The bitwise and of the frame and mask is done so 
	# that only the blue coloured objects are highlighted 
	# and stored in res
	res = cv2.bitwise_and(image,image, mask = mask)
	gray_ = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	cv2.imshow("res", gray_)
else:
	gray_ = gray.copy()

# apply thresholding to single out the brightest spots of the image
gray_thresh = cv2.threshold(gray_, args["thresh"], 255, cv2.THRESH_BINARY)[1]
gray_thresh = cv2.erode(gray_thresh, None, iterations=5)
gray_thresh = cv2.dilate(gray_thresh, None, iterations=5)
cv2.imshow("gray_thresh", gray_thresh)



# apply a Gaussian blur to the image then find the brightest
# region
gray = cv2.GaussianBlur(gray, (args["radius"], args["radius"]), 0)
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
image = orig.copy()
cv2.circle(image, maxLoc, args["radius"], (255, 0, 0), 2)

# apply thresholding to single out the brightest spots of the image
#thresh = cv2.threshold(gray, args["thresh"], 255, cv2.THRESH_BINARY)[1]
#thresh = cv2.erode(thresh, None, iterations=5)
#thresh = cv2.dilate(thresh, None, iterations=5)
#cv2.imshow("thresh", thresh)

# find the contours in the mask, then sort them from left to
# right
cnts = cv2.findContours(gray_thresh.copy(), cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if imutils.is_cv2() else cnts[1]
cnts = contours.sort_contours(cnts)[0]

# loop over the contours
for (i, c) in enumerate(cnts):
	# draw the bright spot on the image
	(x, y, w, h) = cv2.boundingRect(c)
	((cX, cY), circle_radius) = cv2.minEnclosingCircle(c)
	#Check for size of contour and filter out too big
	if int(circle_radius) < 18:
        	print(int(cX), int(cY))
		cv2.circle(detect, (int(cX), int(cY)), int(circle_radius),(0, 255, 0), 2)

	

# show the output image
cv2.imshow("Final", detect)
cv2.waitKey(0)


