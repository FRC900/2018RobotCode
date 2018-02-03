'''
Created on 24 mai 2012

@author: jlengrand
'''
import cv2

import numpy as np

def init_video(video_file):
    """
    Given the name of the video, prepares the flux and checks that everything works as attended
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
        cv2.NamedWindow("Vid", cv2.CV_WINDOW_AUTOSIZE)
    else:
        return None

    nFrames = int(  cv2.GetCaptureProperty( my_video, cv2.CV_CAP_PROP_FRAME_COUNT ) )
    while cpt < nFrames:
        for ii in range(frame_inc):
            img = cv2.QueryFrame(my_video)
            cpt + 1
            
        cv2.ShowImage("Vid", img)
        cv2.WaitKey(delay)


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
            cv2.NamedWindow("Vid", cv2.CV_WINDOW_AUTOSIZE)
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

    ####
    # Start processing here 
    ####
    
     # Turns to one channel image
    grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    display_img(grey_img, 100) 
    # Detect brightest point in image :
    
    ## USed to calculate max of histogram
    #hist_size = [64]
    #h_ranges = [0, 255]
    #hist = cv2.CreateHist([64] , cv2.CV_HIST_ARRAY, [[0, 255]], 1)
    #cv2.CalcHist([grey_img], hist)
    #[minValue, maxValue, minIdx, maxIdx] = cv2.GetMinMaxHistValue(hist)
    #print minValue, maxValue, minIdx, maxIdx

    [minVal, maxVal, minLoc, maxLoc] = cv2.minMaxLoc(grey_img)
    print minVal, maxVal, minLoc, maxLoc
    # could use histogram here to find where to stop
    
    # Threshold at 80% 
    margin = 0.8
    thresh = int( maxVal * margin)
    print thresh

    _ ,thresh_img = cv2.threshold(grey_img, thresh, 255, cv2.THRESH_BINARY)

    display_img(thresh_img, delay = 100)

    # Want to get the number of points now
    _, contours, hierarchy = cv2.findContours(thresh_img,  
                               mode=cv2.RETR_EXTERNAL , 
                               method=cv2.CHAIN_APPROX_NONE)

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

    print "Finished!"
