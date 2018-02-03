'''
Created on 25 mai 2012

@author: jlengrand
'''
from PIL import Image

import sys

import cv2

import numpy as np


# ---- Useful functions ----

def init_video(video_file):
    """
    Given the name of the video, prepares the stream and checks that everything works as attended
    """
    capture = cv2.CaptureFromFile(video_file)

    nFrames = int(  cv2.GetCaptureProperty( capture, cv2.CV_CAP_PROP_FRAME_COUNT ) )
    fps = cv2.GetCaptureProperty( capture, cv2.CV_CAP_PROP_FPS )
    if fps != 0:
        waitPerFrameInMillisec = int( 1/fps * 1000/1 )

        print 'Num. Frames = ', nFrames
        print 'Frame Rate = ', fps, ' frames per sec'

        print '----'
        
        return capture
    else:
        return None

def display_img(img, delay=1000):
    """
    One liner that displays the given image on screen
    """
    cv2.namedWindow("Vid", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("Vid", img)
    cv2.waitKey(delay)

def display_video(my_video, frame_inc=100, delay=100):
    """
    Displays frames of the video in a dumb way.
    Used to see if everything is working fine
    my_video = cv2Capture object
    frame_inc = Nmber of increments between each frame displayed
    delay = time delay between each image 
    """
    cpt = 0    
    img = cv2.QueryFrame(my_video)

    if img != None:
        cv2.namedWindow("Vid", cv2.WINDOW_AUTOSIZE)
    else:
        return None

    nFrames = int(  cv2.GetCaptureProperty( my_video, cv2.CV_CAP_PROP_FRAME_COUNT ) )
    while cpt < nFrames:
        for ii in range(frame_inc):
            img = cv2.QueryFrame(my_video)
            cpt + 1
            
        cv2.imshow("Vid", img)
        cv2.waitKey(delay)

def grab_images(video_file, frame_inc=100, delay = 100):
    """
    Walks through the entire video and save image for each increment
    """
    my_video = init_video(video_file)
    if my_video != None:
        # Display the video and save evry increment frames
        cpt = 0    
        img = cv2.QueryFrame(my_video)
    
        if img != None:
            cv2.namedWindow("Vid", cv2.CV_WINDOW_AUTOSIZE)
        else:
            return None
    
        nFrames = int(  cv2.GetCaptureProperty( my_video, cv2.CV_CAP_PROP_FRAME_COUNT ) )
        while cpt < nFrames:
            for ii in range(frame_inc):
                img = cv2.QueryFrame(my_video)
                cpt += 1
                
            cv2.ShowImage("Vid", img)
            out_name = "data/output/" + str(cpt) + ".jpg"
            cv2.SaveImage(out_name, img)
            print out_name, str(nFrames)
            cv2.WaitKey(delay)
    else: 
        return None

def to_gray(img):
    """
    Converts the input in grey levels
    Returns a one channel image
    """
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    return grey_img
    
def grey_histogram(img, nBins=64):
    """
    Returns a one dimension histogram for the given image
    The image is expected to have one channel, 8 bits depth
    nBins can be defined between 1 and 255 
    """
    hist_size = [nBins]
    h_ranges = [0, 255]
    hist = cv2.CreateHist(hist_size , cv2.CV_HIST_ARRAY, [[0, 255]], 1)
    cv2.CalcHist([img], hist)

    return hist

def extract_bright(grey_img, histogram=False):
    """
    Extracts brightest part of the image.
    Expected to be the LEDs (provided that there is a dark background)
    Returns a Thresholded image
    histgram defines if we use the hist calculation to find the best margin
    """
    ## Searches for image maximum (brightest pixel)
    # We expect the LEDs to be brighter than the rest of the image
    [minVal, maxVal, minLoc, maxLoc] = cv2.minMaxLoc(grey_img)
    print "Brightest pixel val is %d" %(maxVal)
    
    #We retrieve only the brightest part of the image
    # Here is use a fixed margin (80%), but you can use hist to enhance this one    
    if 0:
        ## Histogram may be used to wisely define the margin
        # We expect a huge spike corresponding to the mean of the background
        # and another smaller spike of bright values (the LEDs)
        hist = grey_histogram(img, nBins=64)
        [hminValue, hmaxValue, hminIdx, hmaxIdx] = cv2.GetMinMaxHistValue(hist) 
        margin = 0# statistics to be calculated using hist data    
    else:  
        margin = 0.8
        
    thresh = int( maxVal * margin) # in pix value to be extracted
    print "Threshold is defined as %d" %(thresh)
    
    _ ,thresh_img = cv2.threshold(grey_img, thresh, 255, cv2.THRESH_BINARY)
    
    return thresh_img

def find_leds(thresh_img):
    """
    Given a binary image showing the brightest pixels in an image, 
    returns a result image, displaying found leds in a rectangle
    """
    
    _, contours, hierarchy = cv2.findContours(thresh_img,
                               mode=cv2.RETR_EXTERNAL , 
                               method=cv2.CHAIN_APPROX_NONE)
    print(len(contours))
    regions = []
    n = 0
    while contours:
	n = n + 1
        pts = [ pt for pt in contours ]
	x,y = [],[]
	for i in pts:
	    for j in i:
		for k in j:
		    x.append(k[0])
		    y.append(k[1])
        min_x, min_y = min(x), min(y)
        width, height = max(x) - min_x + 1, max(y) - min_y + 1
        regions.append((min_x, min_y, width, height))
	
	
	try:        
	    contours = contours.h_next()
	except AttributeError: 
	    pass

	out_img = np.zeros(grey_img.shape, np.uint8)
	
        #out_img = cv2.CreateImage(cv2.GetSize(grey_img), 8, 3)
	if n > 8:
	    break
    for x,y,width,height in regions:
        pt1 = x,y
        pt2 = x+width,y+height
        color = (0,0,255,0)
        cv2.rectangle(out_img, pt1, pt2, color, 2)
	
    return out_img, regions

def leds_positions(regions):
    """
    Function using the regions in input to calculate the position of found leds
    """
    centers = []
    for x, y, width, height in regions:
        centers.append( [x+ (width / 2),y + (height / 2)])

    return centers

if __name__ == '__main__':
    video_file =  "data/MusicLEDBox.avi"
    
    if 0:
        # do once once, create some images out of the video
        grab_images(video_file, frame_inc=100, delay = 100)
        

    img = cv2.imread("../data/output/600.jpg")
    if img != None:
        # Displays the image I ll be working with
        cv2.imshow("image", img)
    else:
        print "IMG not found !"
        sys.exit(0)

    ####
    # Starts image processing here 
    ####
    # Turns to one channel image
    grey_img = to_gray(img)
    display_img(grey_img, 1000) 
    # Detect brightest point in image :
    thresh_img = extract_bright(grey_img)
    display_img(thresh_img, delay = 1000)

    # We want to extract the elements left, and count their number
    led_img, regions = find_leds(thresh_img)
    display_img(led_img, delay=1000)

    centers = leds_positions(regions)

    print "Total number of Leds found : %d !" %(len(centers))
    print "###"
    print "Led positions :"
    print centers

    for c in centers:
        print "x : %d; y : %d" %(c[0], c[1])
    print "###"
    
