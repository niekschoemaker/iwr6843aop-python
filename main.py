from typing import Tuple
import serial
import struct
import binascii
import math as m
import argparse
import time
import sys
import json
import curses

# initiate the parser
verbose = False
parser = argparse.ArgumentParser()
parser.add_argument("-v", "--verbose", help="enable verbose mode", action="store_true")
parser.add_argument("-f", "--file", help="output all data as json to file", metavar="PATH", dest='jsonFile')
parser.add_argument("-r", "--rawdata", help="output raw binary to specified file", metavar="PATH", dest='binFile')
parser.add_argument("-t", "--targets", help="output targets data as json to file", metavar="PATH", dest='targetsFile')
parser.add_argument("-c", "--controlport", help="Change the control com port to use (default: COM11)", metavar="COMPORT", dest="controlPort")
parser.add_argument("-d", "--dataport", help="Change the data com port to use (default: COM12)", metavar="COMPORT", dest="dataPort")
args = parser.parse_args()
jsonFile: TextIOWrapper = None
binFile: TextIOWrapper = None
targetsFile: TextIOWrapper = None

# Serial ports, can be overriden by specifying --dataport PORT and --controlport PORT
dataPort = 'COM12'
controlPort = 'COM11'

try:
    if args.jsonFile:
        jsonFile = open(args.jsonFile, 'a')
        print("JSON data will be writen to: %s" % jsonFile.name)
    if args.targetsFile:
        targetsFile = open(args.targetsFile, 'a')
        print("Targets data will be written to: %s" % targetsFile.name)
    if args.binFile:
        binFile = open(args.binFile, 'ab')
        print("Raw binary will be writen to: %s" % binFile.name)
    if args.verbose:
        verbose = True
        print("Verbose mode enabled")
    if args.dataPort:
        dataPort = args.dataPort
        print("Data port set to {:s}".format(dataPort))
    if args.controlPort:
        controlPort = args.controlPort
        print("Control port set to {:s}".format(controlPort))
except:
    e = sys.exc_info()[0]
    print( "Error while parsing arguments: %s" % e )

print(str(verbose))

time.sleep(1)

# region: classess
class ULong: #uint64
    length: int = 8
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<Q', data, offset)[0]

class UInt: #uint32
    length: int = 4
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<I', data, offset)[0]

class UShort: #uint16
    length: int = 2
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<H', data, offset)[0]

class UByte: #uint8
    length: int = 1
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<B', data, offset)[0]

class Float:
    length: int = 4
    def __init__(self):
        pass
    
    def fromBytes(self, data: bytes, offset: int = 0):
        return struct.unpack_from('<f', data, offset)[0]

#endregion

#region dataTypes

# Initiate the data types, used to convert from CTypes to Python Types
# To convert simply iterate over the values and increase the offset by the length property
frameHeaderStructType: dict = { #52 bytes long
    #'sync':             ULong(), # See syncPatternUINT64 below
    'version':          UInt(),
    'platform':         UInt(),
    'timestamp':        UInt(), # 600MHz clocks
    'packetLength':     UInt(), # In bytes, including header
    'frameNumber':      UInt(), # Starting from 1
    'subframeNumber':   UInt(),
    'chirpMargin':      UInt(), # Chirp Processing margin, in ms
    'frameMargin':      UInt(), # Frame Processing margin, in ms
    'uartSentTime':     UInt(), # Time spent to send data, in ms
    'trackProcessTime': UInt(), # Tracking Processing time, in ms
    'numTLVs':          UShort(), # Number of TLVs in thins frame
    'checksum':         UShort() # Header checksum
}

frameHeaderLengthInBytes: int = 52
tlvHeaderLengthInBytes: int = 8
pointLengthInBytes: int = 20
targetLengthInBytes: int = 40

tlvHeaderStruct: dict = { # 8 bytes
    'type':             UInt(), # TLV object Type
    'length':           UInt() # TLV object Length, in bytes, including TLV header
}


# Point Cloud TLV object consists of an array of points. 
# Each point has a structure defined below
# Type 6
pointStruct: dict = { # 20 bytes
    'range':            Float(), # Range, in m
    'angle':            Float(), # Angel, in rad
    'elev':             Float(), 
    'doppler':          Float(), # Doplper, in m/s
    'snr':              Float()    # SNR, ratio
}
# Target List TLV object consists of an array of targets. 
# Each target has a structure define below
# Type 7
targetStruct: dict = { #40 bytes
    'tid':              UInt(),# Track ID
    'posX':             Float(), # Target position in X dimension, m
    'posY':             Float(), # Target position in Y dimension, m
    'posZ':             Float(),
    'velX':             Float(), # Target velocity in X dimension, m/s
    'velY':             Float(), # Target velocity in Y dimension, m/s
    'velZ':             Float(),
    'accX':             Float(), # Target acceleration in X dimension, m/s2
    'accY':             Float(), # Target acceleration in Y dimension, m/s
    'accZ':             Float()
}

cTypesInfo: dict = {
    'uint64': "<Q",
    'uint32': "<I",
    'uint16': "<H",
    'uint8': "<B"
}

config = [
    "sensorStop\n",
    "flushCfg\n",
    "dfeDataOutputMode 1\n",
    "channelCfg 15 7 0\n",
    "adcCfg 2 1\n",
    "adcbufCfg -1 0 0 1 1\n",
    "profileCfg 0 60.6 82 7 40 0 0 82 1 128 5300 0 0 48\n",
    "chirpCfg 0 0 0 0 0 0 0 1\n",
    "chirpCfg 1 1 0 0 0 0 0 2\n",
    "chirpCfg 2 2 0 0 0 0 0 4\n",
    "frameCfg 0 2 64 0 50 1 0\n",
    "lowPower 0 0\n",
    "cfarCfg -1 0 0 8 4 4 0 7120\n",
    "cfarCfg -1 1 0 4 2 3 0 7120\n",
    "SceneryParam -4.0 4 0.1 8 -4 4\n",
    "GatingParam 3 2 2 2 12\n",
    "AllocationParam 100 120 0.1 20 2 20\n",
    "StateParam 10 5 10 100 5\n",
    "VariationParam 0.2887 0.2887 1 1\n",
    "MaxAcceleration 0.1 0.1 0.1\n",
    "AllocZone 0 1\n",
    "CloudPersistence 0\n",
    "trackingCfg 1 2 250 10 200 100 90\n",
    "sensorStart\n"
]

#endregion


# Initialize serial ports
serialControl = serial.Serial()
serialData = serial.Serial()
serialControl.baudrate = 115200
serialControl.port = controlPort
serialData.baudrate = 921600
serialData.port = dataPort
serialControl.open()
serialData.open()

# Send config to radar device
for configValue in config:
    serialControl.write(configValue.encode())
    echo = serialControl.readline()
    done = serialControl.readline()
    prompt = serialControl.read(11)
    # Print without line ending because the echo a line ending
    print(echo.decode('utf-8'), end='')
    if verbose:
        print(done.decode('utf-8'))
        print(prompt.decode('utf-8'))

# syncPattern, gets send at the start of each and every frame, is used to keep sync between this program and the radar device
syncPattern = b'\x02\x01\x04\x03\x06\x05\x08\x07'
# Simply counter to keep track of number of frames captured
dataCount = 0

serialData.read(8)

try:
    # Initialize the fancy console screen
    stdscr = curses.initscr()
    # Set the console as nodelay, makes getch non-blocking
    stdscr.nodelay(1)
    curses.noecho()
    curses.nocbreak()
    y, x = stdscr.getmaxyx()
    subWindow = stdscr.subpad(5, x, 0,0)
    mainWindow = stdscr.subpad(y-6,x,6,0)
    # Enable scrolling for the main console window
    mainWindow.scrollok(True)
    mainWindow.idlok(True)

    while True:
        numTargets = 0
        numPoints = 0
        # Get input from console, is equal to -1 when no input available
        inputCh = stdscr.getch()
        # If input equals 3 (ctrl+c) exit the program
        if inputCh == 3:
            sys.exit(0)

        # Wait for the serial port to receive the sync pattern
        dataBuffer = bytearray(syncPattern)
        while True:
            # Listen for ctrl+c in case it gets stuck
            inputCh = stdscr.getch()
            if inputCh == 3:
                sys.exit(0)

            currentData = serialData.read(1)
            # Remove last e
            dataBuffer.pop(0)
            dataBuffer += currentData
            if binFile:
                binFile.write(currentData)
            # Break the loop if the data is the same as the syncPattern
            if bytes(dataBuffer) == syncPattern:
                break

        # Initialize the data dictionary
        dataDict = {
            'tlvHeaders': list()
        }
        # offset to keep track of which bytes to read next
        offset: int = 0

        # Decode the header, iterate over every type as defined in frameHeaderStructType and decode it to the corresponding data type
        # Same process is repeated for every other type of data (tlvHeader, pointCloud and target)
        for dataPoint in frameHeaderStructType.items():
            currentData = serialData.read(dataPoint[1].length)
            dataDict[dataPoint[0]] = dataPoint[1].fromBytes(currentData, 0)
            if binFile:
                binFile.write(currentData)

        # length of only the data in bytes, so just the packet without the header, used for reading the whole packet in one go
        dataLength = dataDict['packetLength'] - frameHeaderLengthInBytes
        # Read the remaining data
        data = serialData.read(dataLength)

        targetsData = list()

        for nTlv in range(0, dataDict['numTLVs']):
            tlvData = {}
            # Decode TLV Header
            for dataPoint in tlvHeaderStruct.items():
                tlvData[dataPoint[0]] = dataPoint[1].fromBytes(data, offset)
                offset += dataPoint[1].length

            # Length of the TLV Data in bytes, this is the total length as specified in the tlvHeader minus the tlvHeader
            valueLength = tlvData['length'] - tlvHeaderLengthInBytes

            # Decode TLV Data
            if tlvData['type'] == 6: # type 6: Point Cloud Data
                # Number of points, so the data length divided by the length in bytes of one point cloud data point
                numPoints = m.floor(valueLength/pointLengthInBytes)
                for pointData in pointStruct:
                    # Could add data parsing here, but not necessary since we don't need the pointcloud
                    pass
                offset += valueLength

            if tlvData['type'] == 7: # type 7: Target List
                numTargets = m.floor(valueLength/targetLengthInBytes)
                # Print number of targets to console
                stdscr.addstr(0,0,'numTargets: ' + str(numTargets))
                targets = list()
                # Iterate over all targets
                for i in range(0, numTargets):
                    target = {}
                    # Decode Target data
                    for targetData in targetStruct.items():
                        target[targetData[0]] = targetData[1].fromBytes(data, offset)
                        offset += targetData[1].length

                    targets.append(target)
                tlvData['targets'] = targets
                targetsData.append(targets)

            # type 8 contains some additional data regarding targets, seems to be something to do with stance etc, but shouldn't be necessary
            if tlvData['type'] == 8: # type 8: additional target data
                offset += valueLength

            dataDict['tlvHeaders'].append(tlvData)
        if numTargets > 0:
            # We got a target
            if not verbose:
                mainWindow.addstr(0, 0, 'TargetID\tposition\t\tvelocity\t\tacceleration', curses.A_BOLD)
                # If not verbose, so console is available to be used, print the data in a nice format to the console.
                # The data is in a fixed position based on the target id, this makes it easier to track individual targets by a mere human
                for targets in targetsData:
                    for t in targets:
                        mainWindow.addstr(t['tid'] + 1, 0, "{:d}\t\t{{{: .2f}, {: .2f}, {: .2f}}}\t{{{: .2f}, {: .2f}, {: .2f}}}\t{{{: .2f}, {: .2f}, {: .2f}}}".format(
                            t['tid'],
                            t['posX'], t['posY'], t['posZ'],
                            t['velX'], t['velY'], t['velZ'],
                            t['accX'], t['accY'], t['accZ']
                            ))
                    mainWindow.refresh()
            # Write target data to file as json if specified with --targets PATH
            if targetsFile:
                json.dump(targetsData, targetsFile)
            

        if verbose:
            # Print all the collected data to the console, this spams a lot so locked behind -v argument
            mainWindow.addstr(str(dataDict) + '\n')
            mainWindow.refresh()
        
        dataCount += 1
        
        # Write the raw binary data to file if --rawdata PATH is specified
        if binFile:
            binFile.write(data)

        # Write the raw data as json to file if --file PATH is specified
        if jsonFile:
            json.dump(dataDict, jsonFile, indent=4)
        

        # Check if screen was re-sized (True or False)
        # resize = curses.is_term_resized(y, x)

        # if resize is True:

        #      y, x = stdscr.getmaxyx()
        #      stdscr.resize(y, x)
        #      stdscr.clear()
        #      stdscr.refresh()
        #      subWindow.resize(5, x)
        #      subWindow.clear()
        #      subWindow.refresh()
        #      mainWindow.resize(y-6, x)
        #      mainWindow.clear()
        #      mainWindow.scrollok(True)
        #      mainWindow.addstr(mainWindow.getmaxyx()[0]-1, 0, '')
        #      mainWindow.refresh()
        # Clear the window, this is the easiest way to make sure no old data gets left on the screen
        subWindow.clear()
        subWindow.addstr(0, 0, 'Captured frames: {:d}; Captured frames: {:d}'.format(dataCount, dataDict['frameNumber']))
        subWindow.addstr(1, 0, 'People Count: {:d}'.format(numTargets))
        subWindow.addstr(2, 0, '# points in point cloud: {:d}'.format(numPoints))
        subWindow.refresh()

# When program quits make sure the console is returned back to normal mode
finally:
    curses.echo()
    curses.nocbreak()
    curses.endwin()

# Close the serial connections
serialControl.close()
serialData.close()
