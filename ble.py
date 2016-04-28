import sys
from bluetooth.ble import *
from bluepy.btle import *

import signal, os
import subprocess

import serial

# values for iRobot
connection = None
SERIAL_PORT = "/dev/ttyUSB0"

def connect(port):
    global connection
    print "Trying " + str(port) + "... "
    try:
        connection = serial.Serial(port, baudrate=115200, timeout=1)
        print "Connected!"
    except:
        print "Failed."

# a string interpreted as a byte array
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
    print ' '.join([str(ord(c)) for c in command])

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
    cmd = ""
    for v in command.split():
        cmd += chr(int(v))
    sendCommandRaw(cmd)

def init():
    connect(SERIAL_PORT)
    sendCommandASCII('128')
    sendCommandASCII('131')
    time.sleep(1)

pid = os.getpid()

service = DiscoveryService()
devices = service.discover(2)

subp = None
while (True):
    for addr, name in devices.items():
	print("%s (%s)" % (name,addr))
	print(name)
	if (name=="IoT"):
            p = Peripheral(addr)
            svc = p.getServiceByUUID(UUID("A495FF20-C5B1-4B44-B512-1370F02D74DE"))
            ch1 = svc.getCharacteristics(UUID("A495FF21-C5B1-4B44-B512-1370F02D74DE"))[0]
            ch2 = svc.getCharacteristics(UUID("A495FF22-C5B1-4B44-B512-1370F02D74DE"))[0]
            prevData1 = "NaN"
            prevData2 = "NaN"
	    while (True):
                try:
                    data1 = binascii.b2a_hex(ch1.read())
                    data1 = str(data1)
                    data1 = data1[0:2]
                    if (prevData1 == "NaN"):
                        prevData1 = data1
                    elif (data1 == "00" and prevData1 == "01"):
                        print("Switch on")
                        prevData1 = data1
                        subp = subprocess.Popen(["python", "iot-tennis.py", str(pid)])
                    elif (data1 == "01" and prevData1 == "00"):
                        print("Switch off")
                        prevData1 = data1
                        if subp is not None:
                            subp.terminate()

                    data2 = binascii.b2a_hex(ch2.read())
                    data2 = str(data2)
                    data2 = data2[0:2]
                    if (prevData2 == "NaN"):
                        prevData2 = data2
                    elif (data2 == "00" and prevData2 == "01"):
                        print("Button is not pressed")
                        prevData2 = data2
                    elif (data2 == "01" and prevData2 == "00"):
                        print("Button is pressed")
                        prevData2 = data2
                        if subp is not None:
                            subp.terminate()
                        init()
                        sendCommandASCII('143')
                except KeyboardInterrupt:
                    p.disconnect()
                    sys.exit()


