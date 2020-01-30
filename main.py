# Python implementation for IWR6843
# Data structure and reading of data loosely based on the matlab code from IWR6843 overhead people counting lab from Texas-Instruments

from typing import Tuple, TextIO, BinaryIO
import serial
import struct
import binascii
import math as m
import argparse
import time
import sys
import json
import curses
import math
import _thread
from azure.iot.device import IoTHubDeviceClient, Message

CONNECTION_STRING = "HostName=radar-iot-hub.azure-devices.net;DeviceId=iwr6843-1;SharedAccessKey=JHZprpQEVO5NnUX1ImRanBLfOKheMooVWXjqJEyrjwI="
MSG_TXT = '{{"peopleEntered": {peopleEntered},"peopleLeft": {peopleLeft}}}'

# initiate the parser
verbose = False
parser = argparse.ArgumentParser()
parser.add_argument("-v", "--verbose", help="enable verbose mode", action="store_true")
parser.add_argument("-f", "--file", help="output all data as json to file", metavar="PATH", dest='jsonFile')
parser.add_argument("-r", "--rawdata", help="output raw binary to specified file", metavar="PATH", dest='binFile')
parser.add_argument("-t", "--targets", help="output targets data as json to file", metavar="PATH", dest='targetsFile')
parser.add_argument("-p", "--controlport", help="Change the control com port to use (default: COM11)", metavar="COMPORT", dest="controlPort")
parser.add_argument("-d", "--dataport", help="Change the data com port to use (default: COM12)", metavar="COMPORT", dest="dataPort")
parser.add_argument("-c", "--chirpconfig", help="Specify chirpconfig file (default: mmw_pplcount_demo_default.cfg)", metavar="PATH", dest="configFile")
parser.add_argument("--headless", help="Enable headless mode, disables all console functionality", action="store_true")
args = parser.parse_args()
jsonFile: TextIO = None
binFile: BinaryIO = None
targetsFile: TextIO = None
configFilePath: str = "mmw_pplcount_demo_default.cfg"

# Serial ports, can be overriden by specifying --dataport PORT and --controlport PORT
dataPort = 'COM12'
controlPort = 'COM11'
headless = False

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
    if args.configFile:
        configFilePath = args.configFile
        print("Config file set to {:s}".format(configFilePath))
    if args.headless:
        headless = True
        print("Headless mode enabled")
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
with open(configFilePath, "r") as configFile:
    for configLine in configFile.readlines():
        # Send config value to the control port
        serialControl.write(configLine.encode())
        # Wait for response from control port
        echo = serialControl.readline()
        done = serialControl.readline()
        prompt = serialControl.read(11)
        print(echo.decode('utf-8'), end='')
        if verbose:
            print(done.decode('utf-8'))
            print(prompt.decode('utf-8'))


# syncPattern, gets send at the start of each and every frame, is used to keep sync between this program and the radar device
syncPattern = b'\x02\x01\x04\x03\x06\x05\x08\x07'

# Simply counter to keep track of number of frames captured
dataCount = 0
targetData: dict = dict()
peopleCount = 0
peopleEntered = 0
peopleLeft = 0

def newTarget(target: tuple):
    nTarget, dist, lastTime, startingPosition = False, None, time.time(), None
    oldTarget = targetData.get(target['tid'], None)
    inRoom = None
    
    # If we have older data we want to check the older position and check if this target is still in the room or not
    if oldTarget != None and lastTime - oldTarget['lastTime'] < 5.0:
        inRoom = oldTarget['inRoom']
        x, z = oldTarget['target']['posX'], oldTarget['target']['posZ']
        newX, newZ = target['posX'], target['posZ']
        dist = math.sqrt(math.pow((x - z), 2) + math.pow((newX - newZ), 2))
        startingTime = oldTarget['startingTime']
        startingPosition = oldTarget['startingPos']
        nTarget = False
        global peopleCount, peopleEntered, peopleLeft
        # Assuming the sensor is placed correctly z == 0 should be the wall so if it is more than 0 person is in the room, otherwise he is outside the room
        # 0.5 and -0.5 is used to filter false positives and negatives, also works somewhat to filter out things like doors moving in fron of the sensor and just random targets
        if target['posZ'] > 0.5 and not oldTarget['inRoom']:
            if target['posY'] > 0.30:
                peopleCount += 1
                peopleEntered += 1
            inRoom = True
        elif target['posZ'] < -0.5 and oldTarget['inRoom']:
            if target['posY'] > 0.3:
                peopleCount -= 1
                peopleLeft += 1
            inRoom = False
    else:
        nTarget = True
        dist = -1.0
        startingTime = lastTime
        startingPosition = { 'x': target['posX'], 'y': target['posY'] }
        if target['posZ'] > 0.5:
            inRoom = True
        elif target['posZ'] < -0.5:
            inRoom = False

    
    targetData[target['tid']] = {
        'target': target,
        'firstData': nTarget,
        'distance': dist,
        'lastTime': lastTime,
        'startingTime': startingTime,
        'startingPos': startingPosition,
        'inRoom': inRoom
    }

def iothub_client_init():
    # Create an IoT Hub client
    client = IoTHubDeviceClient.create_from_connection_string(CONNECTION_STRING)
    return client

def WatcherThread():
    client = iothub_client_init()
    global peopleEntered, peopleLeft
    while True:
        time.sleep(10)
        msg_txt_formatted = MSG_TXT.format(peopleLeft=peopleLeft, peopleEntered=peopleEntered)
        message = Message(msg_txt_formatted)

        client.send_message(message)
        print ( "Message successfully sent" )
        peopleLeft = 0
        peopleEntered = 0


# This function keeps a buffer of 8 bytes and compares it with the sync pattern
# When sync pattern is found the function returns to the caller's context
def WaitForSyncPattern():
    # Initialize with the syncPattern
    # since data gets popped before the first compare this never causes it to be true but does ensure the length is the same
    dataBuffer = bytearray(syncPattern)

    # Wait for the serial port to receive the sync pattern
    while True:
        if not headless:
            # Listen for ctrl+c in case it gets stuck
            inputCh = stdscr.getch()
            # inputCh 3: Ctrl+c
            if inputCh == 3:
                sys.exit(0)

        # Read 1 byte of data
        currentData = serialData.read(1)
        # Remove the oldest byte received from the serial port
        dataBuffer.pop(0)
        dataBuffer += currentData
        # If binfile is set (dump file parameter on command line) write raw data to file
        if binFile:
            binFile.write(currentData)
        
        # Break the loop if the data is the same as the syncPattern
        if bytes(dataBuffer) == syncPattern:
            return

# Create watcher thread, can be used to send data to ie. a backend
try:
   _thread.start_new_thread( WatcherThread, () )
except Exception as e:
    print(e)
    print("Error: unable to start thread")

try:
    if not headless:
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
        if not headless:
            # Get input from console, is equal to -1 when no input available
            inputCh = stdscr.getch()
            # If input equals 3 (ctrl+c) exit the program
            if inputCh == 3:
                sys.exit(0)

        WaitForSyncPattern()

        # Initialize the data dictionary
        dataDict = {
            'tlvHeaders': list()
        }
        # offset in bytes in the current packet, to ensure the correct bytes are read
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

        currentTargetData = list()

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
                if not headless:
                    # Print number of targets to console
                    stdscr.addstr(0,0,'numTargets: ' + str(numTargets))
                targets = list()
                # Iterate over all targets
                for i in range(0, numTargets):
                    target = {}
                    # Decode Target data
                    for t in targetStruct.items():
                        target[t[0]] = t[1].fromBytes(data, offset)
                        offset += t[1].length

                    targets.append(target)
                    currentTargetData.append(target)
                tlvData['targets'] = targets

            # type 8 contains some additional data regarding targets, seems to be something to do with stance etc, but shouldn't be necessary
            if tlvData['type'] == 8: # type 8: additional target data
                offset += valueLength

            dataDict['tlvHeaders'].append(tlvData)

        if numTargets > 0:
            # This packet contains at least one target
            for t in currentTargetData:
                newTarget(t)
            
            # Write target data to file as json if specified with --targets PATH
            if targetsFile:
                json.dump(currentTargetData, targetsFile)
        
        if not headless:
            # Add the parsed data to the console window in a nice formot
            mainWindow.clear()
            mainWindow.addstr(0, 0, 'TargetID\tposition\t\tvelocity\t\tacceleration\t\tdTime\tdistance', curses.A_BOLD)
            currentTime: float = time.time()
            row = 0
        
            for target in targetData.values():
                if time.time() - target['lastTime'] > 5.0:
                    continue

                t = target['target']
                startPos = target['startingPos']
                row = row + 1
                mainWindow.addstr(row, 0, "{:d}\t\t{{{: .2f}, {: .2f}, {: .2f}}}\t{{{: .2f}, {: .2f}, {: .2f}}}\t{{{: .2f}, {: .2f}, {: .2f}}}\t{: .2f}\t{: .2f}\t{{{: .2f}, {: .2f}}}\t{}".format(
                    t['tid'],
                    t['posX'], t['posY'], t['posZ'],
                    t['velX'], t['velY'], t['velZ'],
                    t['accX'], t['accY'], t['accZ'],
                    currentTime - target['lastTime'], target['distance'],
                    startPos['x'], startPos['y'], str(target['inRoom']) + ' '
                    ))

            mainWindow.refresh()


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
        if not headless:
            subWindow.clear()
            subWindow.addstr(0, 0, 'Captured frames: {:d}; Captured frames: {:d}'.format(dataCount, dataDict['frameNumber']))
            subWindow.addstr(1, 0, 'People Count: {:d}'.format(peopleCount))
            subWindow.addstr(2, 0, '# points in point cloud: {:d}'.format(numPoints))
            subWindow.refresh()


# When program quits make sure the console is returned back to normal mode
finally:
    if not headless:
        curses.echo()
        curses.nocbreak()
        curses.endwin()

# Close the serial connections
serialControl.close()
serialData.close()
