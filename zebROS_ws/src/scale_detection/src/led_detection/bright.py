# USAGE
# python bright.py --image images/both.jpg --radius 61 --thresh 41
# python bright.py --image images/red.jpg --radius 61 --thresh 41
# python bright.py --image images/blue.jpg --radius 61 --thresh 41

# import the necessary packages
from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2

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

# apply thresholding to single out the brightest spots of the image
thresh = cv2.threshold(gray, args["thresh"], 255, cv2.THRESH_BINARY)[1]
thresh = cv2.erode(thresh, None, iterations=5)
thresh = cv2.dilate(thresh, None, iterations=5)
cv2.imshow("thresh", thresh)

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
cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
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


