# Import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import cv2.cv as cv
import numpy as np
import math
import struct
import sys, glob

import signal, os
import subprocess

def termHandler(signum, frame):
    print "Term signal"
    driveForward(0, 0)
    connection = None
    sys.exit()

signal.signal(signal.SIGTERM, termHandler)

try:
    import serial
except ImportError:
    print('Import error', 'Please install pyserial.')
    raise

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320, 240))

# values for iRobot
connection = None
SERIAL_PORT = "/dev/ttyUSB0"

# iRobot Create Commands
DRIVE = 137
WAIT_FOR_DISTANCE = 156
WAIT_FOR_ANGLE = 157
STRAIGHT = 32768
CLOCKWISE = 65535
COUNTER_CLOCKWISE = 1
VELOCITYCHANGE = 200
ROTATIONCHANGE = 300

def connect(port):
    global connection
    print "Trying " + str(port) + "... "
    try:
        connection = serial.Serial(port, baudrate=115200, timeout=1)
        print "Connected!"
    except:
        print "Failed."

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
    cmd = ""
    for v in command.split():
        cmd += chr(int(v))

    sendCommandRaw(cmd)

# sendCommandRaw takes a string interpreted as a byte array
def sendCommandRaw(command):
    global connection

    try:
        if connection is not None:
            connection.write(command)
        else:
            print "Not connected."
    except serial.SerialException:
        print "Lost connection"
        connection = None

    print ' '.join([ str(ord(c)) for c in command ])

def driveForward(velocity, distance):
    driveCmd = struct.pack(">BhH", DRIVE, velocity, STRAIGHT)
    distanceCmd = struct.pack(">BH", WAIT_FOR_DISTANCE, distance)
    sendCommandRaw(driveCmd)
    sendCommandRaw(distanceCmd)

def turn(velocity, angle):
    driveCmd = struct.pack(">BHH", DRIVE, velocity, CLOCKWISE)
    angleCmd = struct.pack(">BH", WAIT_FOR_ANGLE, angle)
    sendCommandRaw(driveCmd)
    sendCommandRaw(angleCmd);

def turnCC(velocity, angle):
    driveCmd = struct.pack(">BHH", DRIVE, velocity, COUNTER_CLOCKWISE)
    angleCmd = struct.pack(">BH", WAIT_FOR_ANGLE, angle)
    sendCommandRaw(driveCmd)
    sendCommandRaw(angleCmd);

def init():
    connect(SERIAL_PORT)
    sendCommandASCII('128')
    sendCommandASCII('131')

def test(pixels):
    ratio = 8.0 / 77.0
    return pixels * ratio

def detectBall():
    kernel = np.ones((5,5),np.uint8)
    
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

    frameCount = 0
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        
        #converting to HSV
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        hue,sat,val = cv2.split(hsv)

        # get info from track bar and appy to result
        '''
        hmn = 12
        hmx = 57
        
        smn = 96
        smx = 255

        vmn = 36
        vmx = 255
        '''
        '''
        hmn = 3
        hmx = 35
        smn = 98
        smx = 200
        vmn = 33
        vmx = 255
        '''
        hmn = 6
        hmx = 57
        smn = 92
        smx = 200
        vmn = 50
        vmx = 200
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
        
        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(closing,cv.CV_HOUGH_GRADIENT,2,120,param1=120,param2=50,minRadius=10,maxRadius=0)
        # circles = np.uint16(np.around(circles))

        countTotal = countTotal + 1
        
        #Draw Circles
        if circles is not None:
                for i in circles[0,:]:
                    #print i
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
                        #print "result: " + str([avgX, avgY, avgR])
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
                        # distance to ball in cm
                        d = -0.2299*np.average(result[2])+32.329
                        # angle to ball in radian
                        testx = test(np.average(result[0]) - 160)
                        angle = math.asin(testx / d)
                        #angle =  math.atan((np.average(result[0])-160)/np.average(result[1])) #*57.2958                        
                        print "RESULT[0]: " + str(np.average(result[0]) - 160)
                        result = [[], [], []]
                        print "\ndistance: "+str(d)+"\nangle: "+str(angle*57.2958)
                        rawCapture.truncate(0)
                        #return d, angle
                        angleD = angle * 57.2958
                        return d, angleD

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        frameCount = frameCount + 1
        if (frameCount > 100):
            break
    return -1, 0


def angleToTime(angle):
    ratio = 1.1 / 90
    return angle * ratio

def dToTime(d):
    ratio = 1.0 / 20.0
    return d * ratio

def move(d, angle):
    x = math.sin(angle)*d
    y = math.cos(angle)*d
    print "x: "+str(x)+"\ny: "+str(y)+"\n"
    driveForward(-1*y*10, 0)
    time.sleep(1)
    if angle==0:
        driveForward(0, 0)
    elif angle < 0:
        turnCC(180, 0) #137 0 180 0 1 157 0 0
        time.sleep(1.1)
    else:
        turn(180, 0) #137 0 180 255 255 157 0 0
        time.sleep(1.1)
    driveForward(-1*x*10, 0)
    time.sleep(1)
    # Stop
    driveForward(0, 0)


def move2(d, angle):
    print "Angle: " + str(angle)
    print "angleToTime: " + str(angleToTime(angle))
    if angle < 0:
        turnCC(180, 0)
    else:
        turn(180, 0)
    time.sleep(abs(angleToTime(angle)))
    driveForward(0, 0)
    print "d: " + str(d)
    print "dToTime: " + str(dToTime(d))
    driveForward(-200, 0)
    time.sleep(dToTime(d))
    driveForward(0, 0)

init()
while True:
    try:
        d, angle = detectBall()
        print d, angle
        if (d < 0):
            turnCC(180, 0)
            time.sleep(angleToTime(30))
            driveForward(0, 0)
        else:
            move2(d, angle)
    except KeyboardInterrupt:
        break


camera.close()
