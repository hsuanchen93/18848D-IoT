import struct
import sys, glob # for listing serial ports
import time
import math

try:
    import serial
except ImportError:
    print('Import error', 'Please install pyserial.')
    raise

connection = None

SERIAL_PORT = "/dev/ttyUSB0"
#SERIAL_PORT = "/dev/tty.usbserial-DA01NZOS"

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

# getDecodedBytes returns a n-byte value decoded using a format string.
# Whether it blocks is based on how the connection was set up.
def getDecodedBytes(n, fmt):
    global connection
    
    try:
        return struct.unpack(fmt, connection.read(n))[0]
    except serial.SerialException:
        print "Lost connection"
        tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
        connection = None
        return None
    except struct.error:
        print "Got unexpected data from serial port."
        return None

# get8Unsigned returns an 8-bit unsigned value.
def get8Unsigned(self):
    return getDecodedBytes(1, "B")

# get8Signed returns an 8-bit signed value.
def get8Signed(self):
    return getDecodedBytes(1, "b")

# get16Unsigned returns a 16-bit unsigned value.
def get16Unsigned(self):
    return getDecodedBytes(2, ">H")

# get16Signed returns a 16-bit signed value.
def get16Signed(self):
    return getDecodedBytes(2, ">h")

def driveForward(velocity, distance):
    driveCmd = struct.pack(">BHH", DRIVE, velocity, STRAIGHT)
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

def square():
    for i in xrange(4):
        turnCC(300, 90)

def test():
    sendCommandASCII('128')
    sendCommandASCII('131')
    time.sleep(1)
    sendCommandASCII('140 3 1 64 16 141 3')
    time.sleep(1)
    square()

# Connect to the Roomba via serial
# Send commands to put Roomba into controllable state
def init():
    connect(SERIAL_PORT)
    sendCommandASCII('128')
    sendCommandASCII('131')
    time.sleep(1)

init()

# forward 10 cm (velocity=mm/s)
#driveForward(100, 0)
#time.sleep(1)

# left turn 90 degrees
#turnCC(180, 0) #137 0 180 0 1 157 0 0
#time.sleep(1.1)
#turn(180, 0) #137 0 180 255 255 157 0 0
#time.sleep(1.1)


driveForward(260, 0)
time.sleep(1)
turnCC(180, 0)
time.sleep(1.1)
driveForward(260, 0)
time.sleep(1)


print "Stopping..."
driveForward(0, 0)
