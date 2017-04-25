import serial
import sys,time
import signal
from time import ctime,sleep
import glob,struct
from multiprocessing import Process,Manager,Array
import threading
import Queue
#import hid

class mSerial():
    ser = None
    def __init__(self):
        print self

    def start(self, port):
        self.ser = serial.Serial(port,115200)
    
    def device(self):
        return self.ser

    def serialPorts(self):
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
        result = []
        for port in ports:
            s = serial.Serial()
            s.port = port
            s.close()
            result.append(port)
        return result

    def writePackage(self, package):
        self.ser.write(package)
        sleep(0.01)

    def read(self):
        return self.ser.read()

    def isOpen(self):
        return self.ser.isOpen()

    def inWaiting(self):
        return self.ser.inWaiting()

    def close(self):
        self.ser.close()
        
class mHID():
    def __init__(self):
        print self
        
    def start(self):
        
        self.manager = Manager()
        self.dict = self.manager.dict()
        self.dict.device = hid.device()
        self.dict.device.open(0x0416, 0xffff)
        #self.dict.device.hid_set_nonblocking(self.device,1)
        print "start"
        self.buffer = []
        self.bufferIndex = 0
    
    def enumerate(self):
        print "enumerate"
        for dev in self.dict.device.enumerate():
            print '------------------------------------------------------------'
            print dev.description()

    def writePackage(self,package):
        buf = []
        buf += [0, len(package)]
        for i in range(len(package)):
            buf += [package[i]]
        n = self.dict.device.write(buf)
        sleep(0.01)

    def read(self):
        c = self.buffer[0]
        self.buffer = self.buffer[1:]
        return unichr(c)
        
    def isOpen(self):
        return True
        
    def inWaiting(self):
        buf = self.dict.device.read(64)
        l = 0
        if len(buf)>0:
            l = buf[0]
        if l>0:
            for i in range(0,l):
                self.buffer += [buf[i+1]]
        return len(self.buffer)
        
    def close(self):
        self.dict.device.close()

class device:
    def __init__(self, name):
        self.name = name
        self.value = 0
        self.callback = None
        self.readPending = False
        self.writePending = False

    def updateValue(self, value):
        self.value = value
        if self.callback is not None:
            self.callback(value)

    def flagReadDone(self):
        self.readPending = False

    def flagWriteDone(self):
        self.writePending = False

    def flagReqFailed(self):
        self.readPending = False
        self.writePending = False
        
class request:
    def __init__(self, device, pack):
        self.device = device
        self.txBuffer = pack    
        
class mBot():
    def __init__(self):
        print "init mBot"
        signal.signal(signal.SIGINT, self.exit)
  
        self.buffer = []
        self.bufferIndex = 0
        self.isParseStart = False
        self.exiting = False
        self.isParseStartIndex = 0
        # Wanted to use Priority queue to give priority to writes, but doesn't work -
        # - Seems to prioritise on request itself instead of priority value
        #self.reqQueue = Queue.PriorityQueue()
        self.readReqQueue = Queue.Queue()
        self.writeReqQueue = Queue.Queue()        
        self.readInProgress = False
        self.writeInProgress = False
        self.reqStartTime = 0

        self.rgbLed = device("rgbLed")
        self.motor = device("motor")
        self.move = device("move")
        self.servo = device("servo")
        self.buzzer = device("buzzer")
        self.sevSegDisplay = device("sevSegDisplay")

        self.lightOnBoard = device("lightOnBoard")            
        self.light = device("light")
        self.buttonOnBoard = device("buttonOnBoard")
        self.iROnBoard = device("iROnBoard")
        self.ultrasonicSensor = device("ultrasonicSensor")
        self.lineFollower = device("lineFollower")

        self.gridX = device("gridX")
        self.gridY = device("gridY")
        self.gridHeading = device("gridHeading")
        self.gridTravel = device("gridTravel")
        self.gridTurn = device("gridTurn")
        self.gridStatus = device("gridStatus")
        self.gridData = device("gridData")
        
        print "Done Init"        
        
    def startWithSerial(self, port):
        self.comDevice = mSerial()
        self.comDevice.start(port)
        self.start()
    
    def startWithHID(self):
        self.comDevice = mHID()
        self.comDevice.start()
        self.start()
    
    def excepthook(self, exctype, value, traceback):
        self.close()
        
    def start(self):
        sys.excepthook = self.excepthook
        #th = threading.Thread(target=self.__commsThread, args=(self.onParse,))
        th = threading.Thread(target=self.__commsThread)
        th.start()
        
    def close(self):
        self.comDevice.close()
        
    def exit(self, signal, frame):
        self.exiting = True
        sys.exit(0)

    def exit(self):
        self.exiting = True
        sys.exit(0)
        
    def __commsThread(self):
        while 1:                         
            if(self.exiting==True):
                break
            if self.readInProgress or self.writeInProgress:
                if ((time.clock() - self.reqStartTime) > 0.1):
                    if self.readInProgress:
                        msg = "Read Request Timed Out "
                    if self.writeInProgress:
                        msg = "Write Request Timed Out "                    
                    print msg + self.currentRequest.device.name
                    
                    self.currentRequest.device.flagReqFailed()
                    self.readInProgress = False
                    self.writeInProgress = False                 
            else:
                try:                    
                    self.currentRequest = self.writeReqQueue.get(False)
                    self.writeInProgress = True
                except:
                    try:
                        self.currentRequest = self.readReqQueue.get(False)
                        self.readInProgress = True
                    except:
                        self.currentRequest = None

                if self.currentRequest is not None:
                    self.comDevice.writePackage(self.currentRequest.txBuffer)
                    #self.__printBuffer("Tx Data:", self.currentRequest.txBuffer)           
    
                    self.reqStartTime = time.clock()
            try:    
                if self.comDevice.isOpen()==True:
                    n = self.comDevice.inWaiting()
                    for i in range(n):
                        r = ord(self.comDevice.read())
                        self.__rxByte(r)
                    sleep(0.01)
                else:   
                    sleep(0.5)
            except Exception,ex:
                print str(ex)
                self.close()
                sleep(1)

    def __rxByte(self, byte):        
        
        position = 0
        value = 0   
        self.buffer+=[byte]
        bufferLength = len(self.buffer)
        if bufferLength >= 2:
            if (self.buffer[bufferLength-1]==0x55 and self.buffer[bufferLength-2]==0xff):
                self.isParseStart = True
                self.isParseStartIndex = bufferLength-2 
            if (self.buffer[bufferLength-1]==0xa and self.buffer[bufferLength-2]==0xd and self.isParseStart==True):         
                self.isParseStart = False

                #self.__printBuffer("Rxed Data:", self.buffer)

                if bufferLength > 4:
                    position = self.isParseStartIndex + 2
                    extID = self.buffer[position]
                    position += 1
                    valType = self.buffer[position]
                    position += 1
                    # 1 byte 2 float 3 short 4 len+string 5 double
                    if valType == 1:
                        value = self.buffer[position]
                    if valType == 2:
                        value = self.readFloat(position)
                        if(value < -255 or value > 1023):
                            value = 0
                    if valType == 3:
                        value = self.readShort(position)
                    if valType == 4:
                        value = self.readString(position)
                    if valType == 5:
                        value = self.readDouble(position)
                    if valType <= 5:
                        self.currentRequest.device.updateValue(value)
                    else:
                        print "Unknown data type " + str(valType)
                    
                if self.writeInProgress:
                    self.currentRequest.device.flagWriteDone()
                    self.writeInProgress = False

                if self.readInProgress:
                    self.currentRequest.device.flagReadDone()
                    self.readInProgress = False

                self.buffer = []

                # Short delay after a request completes before we start the next one, so we don't
                # overload arduino comms 
                sleep(0.1)

    def readFloat(self, position):
        v = [self.buffer[position], self.buffer[position+1],self.buffer[position+2],self.buffer[position+3]]
        return struct.unpack('<f', struct.pack('4B', *v))[0]
    def readShort(self, position):
        v = [self.buffer[position], self.buffer[position+1]]
        return struct.unpack('<h', struct.pack('2B', *v))[0]
    def readString(self, position):
        l = self.buffer[position]
        position+=1
        s = ""
        for i in Range(l):
            s += self.buffer[position+i].charAt(0)
        return s
    def readDouble(self, position):
        v = [self.buffer[position], self.buffer[position+1],self.buffer[position+2],self.buffer[position+3]]
        return struct.unpack('<f', struct.pack('4B', *v))[0]

    def float2bytes(self,fval):
        val = struct.pack("f",fval)
        return [ord(val[0]),ord(val[1]),ord(val[2]),ord(val[3])]

    def short2bytes(self,sval):
        val = struct.pack("h",sval)
        return [ord(val[0]),ord(val[1])]

    def __queueReadRequest(self, device, pack, callback):
        if not device.readPending:
            device.readPending = True
            device.callback = callback
            newReq = request(device, pack)
            self.readReqQueue.put(newReq)

        # If no callback function supplied, wait until read completes and
        # then return value
        if callback is None:        
            while device.readPending:
                pass
            return device.value
        else:
        # Otherwise, return None - value will be returned via callback function
            return None

    def __queueWriteRequest(self, device, pack):
        if not device.writePending:
            device.writePending = True
            newReq = request(device, pack)
            self.writeReqQueue.put(newReq)
            
    def __printBuffer(self, msg, buf):        
        for i in range(0, len(buf)):
            msg = msg + "  " + str(buf[i])           
        print msg   

    def doRGBLed(self, port, slot, index, red, green, blue):
        self.__queueWriteRequest(self.rgbLed, bytearray([0xff, 0x55, 0x9, 0x0, 0x2, 0x8, port, slot, index, red, green, blue]))

    def doRGBLedOnBoard(self, index, red, green, blue):
        self.doRGBLed(0x7, 0x2, index, red, green, blue)

    def doMotor(self, port, speed):
        self.__queueWriteRequest(self.motor, bytearray([0xff,0x55,0x6,0x0,0x2,0xa,port] + self.short2bytes(speed)))

    def doMove(self, leftSpeed, rightSpeed):
        self.__queueWriteRequest(self.move, bytearray([0xff,0x55,0x7,0x0,0x2,0x5] + self.short2bytes(-leftSpeed) + self.short2bytes(rightSpeed)))
        
    def doServo(self, port, slot, angle):
        self.__queueWriteRequest(self.servo, bytearray([0xff,0x55,0x6,0x0,0x2,0xb,port,slot,angle]))
    
    def doBuzzer(self,buzzer,time=0):
        self.__queueWriteRequest(self.buzzer, bytearray([0xff,0x55,0x7,0x0,0x2,0x22] + self.short2bytes(buzzer) + self.short2bytes(time)))

    def doSevSegDisplay(self,port,display):
        self.__queueWriteRequest(self.sevSegDisplay, bytearray([0xff,0x55,0x8,0x0,0x2,0x9,port] + self.float2bytes(display)))
        
    def doIROnBoard(self, message):        
        self.__queueWriteRequest(self.iROnBoard, bytearray([0xff,0x55,len(message)+3,0x0,0x2,0xd,message]))
        
    def requestLightOnBoard(self, callback = None):
        extID = 0
        return self.requestLight(extID, 8, callback)
    
    def requestLight(self, port, callback = None):
        extID = 0
        return self.__queueReadRequest(self.light, bytearray([0xff,0x55,0x4,extID,0x1,0x3,port]), callback)

    def requestButtonOnBoard(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.buttonOnBoard, bytearray([0xff,0x55,0x4,extID,0x1,0x1f,0x7]), callback)
        
    def requestIROnBoard(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.iROnBoard, bytearray([0xff,0x55,0x3,extID,0x1,0xd]), callback)
        
    def requestUltrasonicSensor(self, port, callback = None):
        extID = 0
        return self.__queueReadRequest(self.ultrasonicSensor, bytearray([0xff,0x55,0x4,extID,0x1,0x1,port]), callback)
        
    def requestLineFollower(self, port, callback = None):
        extID = 0
        return self.__queueReadRequest(self.lineFollower, bytearray([0xff,0x55,0x4,extID,0x1,0x11,port]), callback) 

# Grid Follower Interface

    def requestGridX(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridX, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xc8]), callback)        

    def requestGridY(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridY, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xc9]), callback)      

    def requestGridHeading(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridHeading, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xca]), callback)      

    def requestGridTravel(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridTravel, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xcb]), callback)

    def requestGridTurn(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridTurn, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xcc]), callback)

    def requestGridStatus(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridStatus, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xcd]), callback)

    def requestGridData(self, callback = None):
        extID = 0
        return self.__queueReadRequest(self.gridData, bytearray([0xff, 0x55, 0x3, extID, 0x1, 0xce]), callback)    

    def doGridX(self, x):
        self.__queueWriteRequest(self.gridX, bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0xc8] + self.short2bytes(x)))
        
    def doGridY(self, y):
        self.__queueWriteRequest(self.gridY, bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0xc9] + self.short2bytes(y)))

    def doGridHeading(self, heading):
        self.__queueWriteRequest(self.gridHeading, bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0xca] + self.short2bytes(heading)))

    def doGridTravel(self, travelDistance):
        self.__queueWriteRequest(self.gridTravel, bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0xcb] + self.short2bytes(travelDistance)))

    def doGridTurn(self, turnAngle):
        self.__queueWriteRequest(self.gridTurn, bytearray([0xff, 0x55, 0x5, 0x0, 0x2, 0xcc] + self.short2bytes(turnAngle)))
