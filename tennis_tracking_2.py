# Import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cv2.cv as cv
import numpy as np
import math

kernel = np.ones((5,5),np.uint8)

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
#camera.resolution = (640, 480)
camera.resolution = (320, 240)
camera.framerate = 32
#rawCapture = PiRGBArray(camera, size=(640, 480))
rawCapture = PiRGBArray(camera, size=(320, 240))

# initialize variables
count = 0
countTotal = 0
sumR = 0
sumX = 0
sumY = 0
avgR = -1
avgX = -1
avgY = -1
result = [[], [], []]

# allow the camera to warmup
time.sleep(0.1)

def nothing(x):
    pass
# Creating a windows for later use
cv2.namedWindow('HueComp')
cv2.namedWindow('SatComp')
cv2.namedWindow('ValComp')
cv2.namedWindow('closing')
cv2.namedWindow('tracking')

# Creating track bar for min and max for hue, saturation and value
# You can adjust the defaults as you like
cv2.createTrackbar('hmin', 'HueComp',20,179,nothing)
cv2.createTrackbar('hmax', 'HueComp',57,179,nothing)

cv2.createTrackbar('smin', 'SatComp',148,255,nothing)
cv2.createTrackbar('smax', 'SatComp',200,255,nothing)

cv2.createTrackbar('vmin', 'ValComp',50,255,nothing)
cv2.createTrackbar('vmax', 'ValComp',200,255,nothing)

# My experimental values
# hmn = 12
# hmx = 37
# smn = 145
# smx = 255
# vmn = 186
# vmx = 255

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):   
    buzz = 0
    image = frame.array

    #converting to HSV
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    hue,sat,val = cv2.split(hsv)

    # get info from track bar and appy to result
    hmn = cv2.getTrackbarPos('hmin','HueComp')
    hmx = cv2.getTrackbarPos('hmax','HueComp')
    
    smn = cv2.getTrackbarPos('smin','SatComp')
    smx = cv2.getTrackbarPos('smax','SatComp')


    vmn = cv2.getTrackbarPos('vmin','ValComp')
    vmx = cv2.getTrackbarPos('vmax','ValComp')

    # Apply thresholding
    hthresh = cv2.inRange(np.array(hue),np.array(hmn),np.array(hmx))
    sthresh = cv2.inRange(np.array(sat),np.array(smn),np.array(smx))
    vthresh = cv2.inRange(np.array(val),np.array(vmn),np.array(vmx))

    # AND h s and v
    tracking = cv2.bitwise_and(hthresh,cv2.bitwise_and(sthresh,vthresh))

    # Some morpholigical filtering
    dilation = cv2.dilate(tracking,kernel,iterations = 1)
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    closing = cv2.GaussianBlur(closing,(5,5),0)

    '''
    # Blob Detection71630857]
ball at 11.4853844257centimeters

    # Set up the detector with default parameters
    detector = cv2.SimpleBlobDetector_create()
    # Detect blobs
    keypoints = detector.detect(image)
    # Draw detected blobs as red circles
    image_keypts = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255))
    '''
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(closing,cv.CV_HOUGH_GRADIENT,2,120,param1=120,param2=50,minRadius=10,maxRadius=0)
    # circles = np.uint16(np.around(circles))

    countTotal = countTotal + 1
    
    #Draw Circles
    if circles is not None:
            for i in circles[0,:]:
                print i
                # get initial radius to compare to
                if avgR == -1:
                    avgX = i[0]
                    avgY = i[1]
                    avgR = i[2]
                # compare radius to average value and get new average
                if i[2]>avgR-13 and i[2]<avgR+13:
                    count = count + 1
                    sumX = sumX + i[0]
                    sumY = sumY + i[1]
                    sumR = sumR + i[2]
                    avgX = sumX/float(count)
                    avgY = sumY/float(count)
                    avgR = sumR/float(count)
                # the initial value isn't detected again
                if countTotal>=15 and count<7:
                    count = 0
                    countTotal = 0
                    sumR = 0
                    sumX = 0
                    sumY = 0
                    avgR = -1
                    avgX = -1
                    avgY = -1
                    result = [[], [], []]
                # find the ball once
                elif count >= 7:
                    print "result: " + str([avgX, avgY, avgR])
                    result[0].append(avgX)
                    result[1].append(avgY)
                    result[2].append(avgR)
                    count = 0
                    countTotal = 0
                    sumR = 0
                    sumX = 0
                    sumY = 0
                    avgR = -1
                    avgX = -1
                    avgY = -1
                # find the ball 3 times before calculating the distance and angle
                if len(result[0])==3:
                    print "\nball at "+str(-0.2299*np.average(result[2])+32.329)+" cm"
                    r = math.atan((np.average(result[0])-160)/np.average(result[1]))
                    print "ball at "+str(r*57.2958)+" degrees\n"
                    result = [[], [], []]
                    
                # If the ball is far, draw it in green    
                if int(round(i[2])) < 30:
                    cv2.circle(image,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
                    cv2.circle(image,(int(round(i[0])),int(round(i[1]))),2,(0,255,0),10)
				# else draw it in red
                elif int(round(i[2])) > 35:
                    cv2.circle(image,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,0,255),5)
                    cv2.circle(image,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
                    buzz = 1
    
	#you can use the 'buzz' variable as a trigger to switch some GPIO lines on Rpi :)
    # print buzz                    
    # if buzz:
        # put your GPIO line here

    
    #Show the result in frames
    cv2.imshow('HueComp',hthresh)
    cv2.imshow('SatComp',sthresh)
    cv2.imshow('ValComp',vthresh)
    cv2.imshow('closing',closing)
    cv2.imshow('tracking',image)
    #cv2.imshow('tracking', image_keypts)

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
    
camera.close()
cv2.destroyAllWindows()
