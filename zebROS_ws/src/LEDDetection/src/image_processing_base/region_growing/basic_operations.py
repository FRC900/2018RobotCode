'''
Contains all basic functions, such as image creation, copy, . . .
No Image processing complex algorithms here. 

Created on Nov 19, 2011

@author: Julien Lengrand-Lambert
@email: julien@lengrand.fr
'''

import sys
import cv2

_mouse_pos = []

def _on_mouse(event, x, y, flags, param):
    """
    None = onMouse(event, x, y, flags, param)

    Function automatically called by opencv2 when having mouse events on a 
    displayed frame. In here, we are searching for a left mouse click
    """
    global _mouse_pos
    if event  == cv2.CV_EVENT_LBUTTONDOWN :
        _mouse_pos.append((x, y))

def mouse_point(img, name="Mouse Points", mode="S"):
    """
    Displays input image and waits for user click on a point of the image.
    For each click, the (x, y) coordinates are retrieved.
    
    Two modes are possible:
    -Single : 
        The function exits automatically on first click.
        Returns a list containing 1 (x,y) tuple.
    -Multiple :
        All clicks are saved. The function exists when the user types on keyboard
        Returns containing (x,y) tuples.
    ---
    Ex:
    img = cv2.LoadImage("../data/tippy.jpg", cv2.CV_LOAD_IMAGE_GRAYSCALE)
    mouse_points(img, name="mouse points", mode="S")
    """
    
    if not isinstance(name, str):
        raise TypeError("(%s) Name :String expected!" % (sys._getframe().f_code.co_name))
    if not isinstance(mode, str):
        raise TypeError("(%s) Mode :String expected!" % (sys._getframe().f_code.co_name))
        
    global _mouse_pos
    cv2.StartWindowThread()
    cv2.NamedWindow(name, cv2.CV_WINDOW_AUTOSIZE)
    cv2.SetMouseCallback(name, _on_mouse)
    
    try :
        cv2.ShowImage(name, img)
    except TypeError:
        raise TypeError("(%s) img : IplImage expected!" % (sys._getframe().f_code.co_name))

    if mode == "S":
        _single_point()
    elif mode == "M":
        _multiple_points()
    else:
        raise ValueError("(%s) Mode can be either S (single) or M (multiple)" % (sys._getframe().f_code.co_name))
    
    cv2.DestroyWindow(name)
    return _mouse_pos

def _single_point():
    """
    Internal function, used in mouse_point
    """
    while(len(_mouse_pos) == 0):
        cv2.WaitKey(10)

def _multiple_points():
    """
    Internal function, used in mouse_point
    """
    char = -1
    while (char == -1):
        char = cv2.WaitKey(10)   
# ----
def create_same_image(in_img, zero=0):
    """
    Creates an image of same size, depth and number of channels as input image
    If zero is activated, the image content is set to 0 to avoid parasites.
    
    ---
    Ex:
    img = cv2.LoadImage("../data/tippy.jpg", cv2.CV_LOAD_IMAGE_GRAYSCALE)
    copy_img = create_same_image(img, 0)
    """
    try :
        out_img = cv2.CreateImage(cv2.GetSize(in_img), 
                                 in_img.depth, 
                                 in_img.nChannels)
    except TypeError:
        raise TypeError("(%s) Image creation failed!" % (sys._getframe().f_code.co_name))

    if zero:
        cv2.Zero(out_img)
        
    return out_img


