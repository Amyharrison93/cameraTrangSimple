#save this in the same directory as your program and you can use #include 3dPosition to call it

import cv2 as cv
import numpy as np
import math
 
def PxToAngle(lensAngle, ImageDimention):
    """converts the pixel position into an angle 
    lensAngle = float representing the angle of the lens,
    ImageDimention =  integer value representing the selected (x/y) dimention
    returns float: scale"""
    #work out pixel per degree
    scale = ImageDimention/lensAngle

    return scale

def depth(angle1, angle2, camDistance):
    """calculates the distance of a point appearing in two cameras
    angle 1/2 = float representing the angle of the object
    camDistance = the distance between two cameras
    returns float: distance"""
    angle3 = 180-(angle1+angle2)    #work out all angles of triagle
    distance1 = (camDistance*(math.sin(angle2)))/(math.sin(angle3))     #work out all lengths of sides
    distance2 = (camDistance*(math.sin(angle1)))/(math.sin(angle3))     #work out all lengths of sides
    s = (camDistance+distance1+distance2)/2     
    area = math.sqrt(s*(s-camDistance)*(s-distance1)*(s-distance2))     #work out the area
    distance = 2*(area/camDistance)     #work out the height to get the distance away from the origin/point between two cameras

    return distance

def Xposition(angle1, angle2, camDistance):
    """calculates where a point would intersect with the base if split into two right angle triangles
    angle 1/2 = float representing the angle of the object
    camDistance = the distance between two cameras
    returns float: distance"""
    angle3 = 180-(angle1+angle2)    #work out all angles of triagle
    distance1 = (camDistance*(math.sin(angle2)))/(math.sin(angle3))     #work out all lengths of sides
    distance2 = (camDistance*(math.sin(angle1)))/(math.sin(angle3))     #work out all lengths of sides
    s = (camDistance+distance1+distance2)/2     
    area = math.sqrt(s*(s-camDistance)*(s-distance1)*(s-distance2))     #work out the area
    height = 2*(area/camDistance)     #work out the height to get the distance away from the origin/point between two cameras
    distance = camDistance - math.sqrt((height*height)+(distance1*distance1))   #gives the distance from right to left the object is

    return distance

def Yposition(angle1, depth, camHeight):
    """calculates the height of an object in frame
    angle 1/2 = float representing the angle of the object
    camHeight = the height of the cameras from the ground"""
    height = (depth*math.sin(angle1))/math.sin(90)
    height = camHeight+height

    return height

frame1 = np.zeros((1080,1920))    #placeholder for your actual frame
frame2 = np.zeros((1080,1920))    #placeholder for your actual frame

lens1Angle = 40
lens2Angle = 45

camScale1 = PxToAngle(lens1Angle, frame1.shape[0])     #pixel to angle conversion - you will still need an offset heremeasure from mount
camScale2 = PxToAngle(lens2Angle, frame2.shape[0])     #pixel to angle conversion - you will still need an offset heremeasure from mount

cam1Offset = 0      #this is the offset in degrees from the edge of the image to your 0deg mark/centre point
cam2Offset = 0      #this is the offset in degrees from the edge of the image to your 0deg mark/centre point

#angles are equal to their x co-ordinate positionadjusted by the scale factor