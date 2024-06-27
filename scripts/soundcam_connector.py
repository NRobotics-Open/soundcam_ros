#!/usr/bin/env python3

'''
This package is developed by N Robotics GmbH to interface with the Sound Camera 
from CAE Sofware & Systems GmbH.

Developer: ephson@nrobotics.com
'''

import socket, time, atexit, os, io, sys, signal, subprocess
from threading import Thread
from enum import Enum
from bitarray import bitarray
import numpy as np, cv2
np.set_printoptions(threshold=sys.maxsize)

from matplotlib.animation import FuncAnimation, FFMpegWriter, writers
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from soundcam_protocol import CommandCodes, DataMessages, MDDataMessage, Device, \
    DataObjects, \
    CameraProtocol as CP, CameraProtocolProxy as CPP, CameraProtocolManager
import multiprocessing
from multiprocessing import Process, Queue, Semaphore
import SharedArray as sa

from utils import SoundUtils as SU, SoundUtilsProxy as SUP, SoundUtilsManager, \
    EuclideanDistTracker

#http stream
#from soundcam_streamer import StreamingServer, StreamingOutput, StreamingHandler
    

FIRMWARE_VERSION = '2.8.3.0'
CONNECT_TIMEOUT = 25 #seconds

class SoundCamConnector(object):
    class DTypes(Enum):
        GLOBAL = 0
        VID = 1
        ACOUSTIC_VID = 2
        SPECTRUM = 3
        AUDIO = 4
        THERMAL = 5
        RAW = 6

        O_VID = 7
        O_ACVID = 8
        O_SPEC = 9
        O_AUD = 10
        O_THM = 11
        O_BW = 12
    
    class OverlayTypes(Enum):
        ACOUSTIC = 0
        SPECTRUM = 1
        AUDIO = 2
        THERMAL = 3
        MAX = 4

    def __init__(self, deviceIP:str='169.254.169.138', devicePort:int=6340, 
                 protocol:int=3, cfgObj:dict={}, debug:bool = False) -> None:
        print('Firmware Version: ', FIRMWARE_VERSION)
        self.debug = debug
        self.testing = False
        self.invokeId = 1
        self.cfgObj = cfgObj
        self.connected = False

        #debug parameters
        self.new_frame_t = 0
        self.prev_frame_t = 0

        #prepare utils manager
        manager = SoundUtilsManager()
        manager.start()
        self.scamUtils = manager.SoundUtils()
        self.objTracker = EuclideanDistTracker()
        
        self.shmnames=['npaudiomem', 'npglobalspecmem', 'nplocalspecmem', 'npacmem', 'npenergymem']

        self.scamUtils.setScalingMode(SU.ScalingMode(self.cfgObj['scalingMode']), 
                                      dynamic=self.cfgObj['dynamic'], 
                                      max=self.cfgObj['maximum'],
                                      crest= self.cfgObj['crest'])
        for shm in self.shmnames:
            try:
                sa.delete(shm) #audio buffer
            except:
                pass
        self.audioBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.AUDIO.value], (self.scamUtils.p_getAudLen(), 1))
        self.globalSpecBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.GLOBAL_SPECTRUM.value], (self.scamUtils.p_getWindow(), 1023))
        self.localSpecBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.LOCAL_SPECTRUM.value], (self.scamUtils.p_getWindow(), 1023))
        self.energyBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.ENERGY.value], (self.scamUtils.p_getACLen(), 1))
        self.acousticBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.ACOUSTIC.value], (self.scamUtils.p_getACLen(), 1))
        self.scamUtils.initializeSharedBuffers(shmnamesls=self.shmnames)

        #check SoundCam Connectivity and Initialize connection
        # if(not self.pingCamera(deviceIP=self.cfgObj['ip'])):
        #     exit(-99)

        self.bufSize = 1460
        self.recvStream = False
        self.processData = multiprocessing.Event()
        self.processData.set()
        self.visualUp = False
        self.connected = False

        #prepare queues
        self.processes = list()
        self.threads = list()
        self.qDict = dict()

        #post-processing setup
        self.overlayBA = bitarray(self.cfgObj['overlayConfig'])
        self.qDict[SoundCamConnector.DTypes.O_BW.name] = Queue(maxsize=3)
        self.qDict[SoundCamConnector.DTypes.O_VID.name] = Queue(maxsize=3)
        self.qDict[SoundCamConnector.DTypes.O_ACVID.name] = Queue(maxsize=3)
        self.qDict[SoundCamConnector.DTypes.O_SPEC.name] = Queue(maxsize=3)
        self.qDict[SoundCamConnector.DTypes.O_AUD.name] = Queue(maxsize=3)
        self.qDict[SoundCamConnector.DTypes.O_THM.name] = Queue(maxsize=3)
            
        #self.processes.append(Process(target=self.postProc))
        if(not self.cfgObj['system_run']): #prevent any form of visualization if not system_run
            #self.cfgObj['visualizeOverlay'] = False
            self.cfgObj['visualizeVideo'] = False
            self.cfgObj['visualizeThermal'] = False
            self.cfgObj['visualizeAudio'] = False
            self.cfgObj['visualizeAcVideo'] = False
            self.cfgObj['visualizeSpectrum'] = False

        # if(self.cfgObj['visualizeOverlay']):
        #     self.cfgObj['visualizeVideo'] = True
        
        if(self.cfgObj['processVideo']):
            self.vidQ = Queue()
            self.qDict[SoundCamConnector.DTypes.VID.name] = self.vidQ
            #self.threads.append(Thread(target=self.processVideo, args=(self.vidQ, self.processData), daemon=True))
            self.processes.append(Process(target=self.processVideo, args=(self.vidQ, self.processData), name='vidProc'))
        if(self.cfgObj['processAcVideo']):
            self.acVidQ = Queue()
            self.qDict[SoundCamConnector.DTypes.ACOUSTIC_VID.name] = self.acVidQ
            self.ac_semaphore = Semaphore(1)
            self.processes.append(Process(target=self.processAcVideo, 
                                          args=(self.acVidQ, self.processData, 
                                                self.scamUtils, self.ac_semaphore)))
        if(self.cfgObj['processSpectrum']):
            self.specQ = Queue()
            self.qDict[SoundCamConnector.DTypes.SPECTRUM.name] = self.specQ
            self.spec_semaphore = Semaphore(1)
            self.processes.append(Process(target=self.processSpectrum, 
                                          args=(self.specQ, self.processData, 
                                                self.scamUtils, self.spec_semaphore)))
        if(self.cfgObj['processAudio']):
            self.audQ = Queue()
            self.qDict[SoundCamConnector.DTypes.AUDIO.name] = self.audQ
            self.aud_semaphore = Semaphore(1)
            self.processes.append(Process(target=self.processAudio, 
                                          args=(self.audQ, self.processData, 
                                                self.scamUtils, self.aud_semaphore)))
        if(self.cfgObj['processThermal']):
            self.thermalQ = Queue()
            self.qDict[SoundCamConnector.DTypes.THERMAL.name] = self.thermalQ
            self.processes.append(Process(target=self.processThermalVideo, 
                                          args=(self.thermalQ, self.processData)))
        # if(self.cfgObj.p_pubRaw):
        #     self.rawQ = Queue()
        #     self.qDict[SoundCamConnector.DTypes.RAW.name] = self.rawQ
        #     self.processes.append(Process(target=self.publishRaw, args=(self.rawQ, self.processData)))
        
        if(self.cfgObj['visualizeAudio'] or self.cfgObj['visualizeSpectrum']):
            self.processes.append(Process(target=self.displayProcess, args=(self.processData,)))

        #prepare protocol manager
        protocmanager = CameraProtocolManager()
        protocmanager.start()
        self.protocol = protocmanager.CameraProtocol(protocol=self.cfgObj['protocol'], debug=self.debug) #holds & processes received data from device

        #start processes
        print('Starting processes ...')
        self.globalQ = Queue()
        self.qDict[SoundCamConnector.DTypes.GLOBAL.name] = self.globalQ 
        self.processes.append(
            Process(target=self.streamFilter, args=(self.qDict, self.processData, self.protocol)))

        for proc in self.processes:
            proc.start()
        
        #start threads
        self.threads.append(Thread(target=self.receiveCyclic, daemon=True))
        for th in self.threads:
            th.start()

        #Note: All data is sent as Little Endian!
    
    '''Ping Camera'''
    def pingCamera(self, deviceIP):
        #check SoundCam Connectivity and Initialize connection
        #start_t = time.time()
        # while(True):
        #res = os.system("ping -c 1 " + deviceIP)
        proc = subprocess.Popen(["ping", deviceIP, "-c", "1", "-W", "2"], 
                                stdout=subprocess.PIPE)
        proc.wait()
        if(proc.poll() == 0):
            return True
            # if((time.time() - start_t) > CONNECT_TIMEOUT):
            #     print('Camera not connected. Check network configuration!')
            #     break
            # time.sleep(0.5)
        return False

    '''
        Establishes connection with the camera and performs handshake (gets device Id)
    '''
    def connect(self):
        ip_addr = self.cfgObj['ip'] 
        port = self.cfgObj['port']
        self.sock = socket.socket(socket.AF_INET, # IPv4
                        socket.SOCK_STREAM) # UDP SOCK_DGRAM   #TCP SOCK_STREAM
        if hasattr(socket, "SO_REUSEPORT"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        try:
            self.sock.connect((ip_addr, port))
            self.connected = True
        except Exception as ex:
            print('Failure Establishing connection to device! ', ex)
            self.connected = False
            exit(-1)
        print("\nSocket initialized on: \t IP| %s PORT| %d" % (ip_addr, port))
        #Request device Identification
        query = self.protocol.generateReadRequest(command=CommandCodes.IdentificationReq, 
                                                  invokeId=self.invokeId)
        self.invokeId += 1
        query += self.protocol.writeCameraLighting(self.invokeId, 
                                                   self.cfgObj['cameraLighting'])
        self.sendData(query=query)
        return self.connected
    
    ''' REGEX pattern matcher '''
    def getMatch(self, buf):
        match = None
        for ptn in self.protocol.p_getPatterns():
            res = ptn.search(buf)
            if(match is None):
                match = res
            elif(res and (res.span()[0] < match.span()[0])):
                match = res
        return match

    '''Sends command to reset the camera'''
    def restartCamera(self):
        query = self.protocol.generateReadRequest(command=CommandCodes.ResetReq, 
                                                  invokeId=self.invokeId)
        self.sendData(query=query)
        return True

    def disconnect(self):
        self.processData.clear()
        if(self.connected):
            try:
                #print('Stopping measurement ...')
                cv2.destroyAllWindows()
                plt.close()
                #self.stopStream()
                self.stopMeasurement()
                self.sock.close()
                #print('done!')
            except Exception as ex:
                print('Error closing connection: ', ex)
        try:
            print('Joining threads/processes ...')
            for th in self.threads:
                th.join()
            for p in self.processes:
                if(p.is_alive()):
                    #print(f'Terminating {p.name}')
                    p.terminate()
                    p.join(timeout=1)  # Wait up to 1 second for process to terminate
                    if(p.is_alive()):
                        #print(f'Force killing {p.name}')
                        os.kill(p.pid, signal.SIGKILL)
                        p.join()
            print('done!')
        except Exception as ex:
            print('Error joining processes: ', ex)
    
    ''' Release shared memory blocks '''
    def release_shared(self):
        for i, shm in enumerate(self.shmnames):
            #self.semaphores[i].acquire()
            sa.delete(shm)
            #self.semaphores[i].release()
        #del self.semaphores

    '''Send data over Network'''
    def sendData(self, query:bytes):
        #check and send
        try:
            if(query is not None):
                written = self.sock.send(query)
                if(self.debug):
                    print("Written: %s \Length: %i" % (query, written))
                self.invokeId += 1
                if(self.invokeId > 0xFF):
                    self.invokeId = 0
                time.sleep(0.001)
            else:
                print("Warning: Empty Query!")
        except Exception as e:
            print('Sending error: ', e) 
    
    '''
        Configures the camera:
            - Sets: 
                distance 
                min & max frequency
                camera resolutions (Width x Height)
                acoustic Image FPS
                acoustic Averaging
            - Sets the device state
            - Sets the data to be sent from the device
    '''
    def initialConfigure(self, getQuery=False)->bool:
        try:
            #Configure with WriteObjectReq request --- multiple
            query = self.protocol.ConfigureCamera(self.invokeId, 
                            self.cfgObj['distance'], 
                            (self.cfgObj['minFrequency'], self.cfgObj['maxFrequency']), 
                            (self.cfgObj['camera_W'], self.cfgObj['camera_H']),
                            self.cfgObj['acimageFPS'], 
                            self.cfgObj['acAvg'])
            #concatenate PrepareState
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, 
                                            Device.DeviceStates(self.cfgObj['deviceState']))#set device state
            #concatenate DataToSend
            self.invokeId += 1
            query += self.protocol.dataToSendConfig(self.invokeId, 
                                                    dataToSend1=self.cfgObj['dataToSend1'],
                                                    dataToSend2=self.cfgObj['dataToSend2'])
            if(getQuery):
                return query
            self.sendData(query=query)
        except Exception as ex:
            print('Error Configuring device!', ex)
            return False
        return True
    
    '''
        Configures the camera:
            - Sets: 
                distance 
                min & max frequency
                camera resolutions (Width x Height)
                acoustic Image FPS
                acoustic Averaging
            - Updates the acoustic filter
    '''
    def updatePreset(self, mode, distance, minFreq, maxFreq,
                     dynamic=3.1, crest=5.0, maximum=None):
        try:
            #Configure with WriteObjectReq request --- multiple
            query = self.protocol.ConfigureCamera(self.invokeId, 
                            distance, (minFreq, maxFreq), 
                            (self.cfgObj['camera_W'], self.cfgObj['camera_H']),
                            self.cfgObj['acimageFPS'], 
                            self.cfgObj['acAvg'])
            #concatenate PrepareState
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, 
                                            Device.DeviceStates(self.cfgObj['deviceState']))#set device state
            self.sendData(query=query)

            #configure acoustic filter
            self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                          max=maximum, crest=crest)
        except Exception as ex:
            print('Error Configuring device!', ex)
            return False
        return True
    
    def startMeasurement(self)->bool:
        try:
            """ query = self.protocol.setState(self.invokeId, Device.DeviceStates.Streaming)#set streaming state
            self.sendData(query=query)
            query = self.protocol.dataToSendConfig(self.invokeId)
            self.sendData(query=query) """
            print('Starting measurement ...')
            #query = self.protocol.writeVidFPS(self.invokeId, self.cfgObj['videoFPS'])
            query = self.protocol.startStopProcedure(self.invokeId) #start procedure
            self.sendData(query=query)
            self.recvStream = True
        except Exception as ex:
            print('Error Starting Measurement!')
            return False
        self.protocol.setStreamFlags()
        return True
    
    def stopMeasurement(self)->bool:
        try:
            print('Stopping measurement ...')
            query = self.protocol.startStopProcedure(self.invokeId, False) #stop procedure
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, Device.DeviceStates.Idle) #set Idle state
            self.sendData(query=query)
            self.protocol.unsetStreamFlags()
            self.recvStream = False
        except Exception as ex:
            print('Error Stopping Measurement: ', ex)
            return False
        return True
    
    def isMeasuring(self):
        return self.protocol.isStreaming()
    
    '''Packs the continuous incoming stream to the generic Queue'''
    def receiveCyclic(self):
        print('Cyclic thread started ...')
        start_t = time.time()
        # heartrate = 1/ self.cfgObj['heartbeatRate']
        # heart_t = time.time()
        commonstatus_t = time.time()
        while(self.processData.is_set()):
            if(self.connected):
                res = self.sock.recv(self.bufSize)
                if(res):
                    if(self.recvStream):
                        self.globalQ.put(res)
                    else: #if not streaming
                        try:
                            self.protocol.unpackDecodeResponse(response=res)
                        except Exception as ex:
                            print('\nError Unpacking and Decoding ...')

                now = time.time()
                if((not self.recvStream) and self.protocol.p_isConfigured()): #query for common status when not receiving stream
                    if((now - commonstatus_t) >= 0.5):
                        query = self.protocol.generateReadRequest(command=CommandCodes.ReadDataObjectReq, invokeId=self.invokeId, dataLs=[DataObjects.Ids.CommonStatus])
                        self.sendData(query)
                        commonstatus_t = now
                
                if((now - start_t) >= 10.0 and self.connected and self.debug):
                    print('Cyclic loop running ...')
                    start_t = time.time()

                """ if((now - heart_t) >= heartrate):
                    #send ACK to device
                    heart_t = now """
                #time.sleep(0.0001)
            else:
                #print('Awaiting socket connnection ...')
                time.sleep(0.1)
        
    '''Filters the generic Deque to the corresponding Deques based on the type'''
    def streamFilter(self, qDict:dict, canRun:multiprocessing.Event, protocol:CP):
        print('Starting stream filter ...')
        canPopfromQ:bool = True
        contRead:bool = False
        curBf = bytes()
        xsData = bytes()
        inObj = bytes()
        curId:int = 0
        remainingLen:int = 0
        
        #get queues
        globalQ = None
        vidQ = None
        acVidQ = None
        specQ = None
        audQ = None
        thermalQ = None
        rawQ = None
        for k,v in qDict.items():
            if (k == SoundCamConnector.DTypes.GLOBAL.name):
                globalQ:Queue = v
            elif(k == SoundCamConnector.DTypes.VID.name):
                vidQ:Queue = v
            elif(k == SoundCamConnector.DTypes.ACOUSTIC_VID.name):
                acVidQ:Queue = v
            elif(k == SoundCamConnector.DTypes.SPECTRUM.name):
                specQ:Queue = v
            elif(k == SoundCamConnector.DTypes.AUDIO.name):
                audQ:Queue = v
            elif(k == SoundCamConnector.DTypes.THERMAL.name):
                thermalQ:Queue = v
            elif(k == SoundCamConnector.DTypes.RAW.name):
                rawQ:Queue = v

            
        
        dmsg:MDDataMessage = None
        isHdrPkt = False
        canPrintLen = True
        # cnt = 0
        while(canRun.is_set()):
            #read queue and filter
            if(not globalQ.empty()):
                # if(globalQ.qsize() > 50):
                #     print('Global Queue Size: ', globalQ.qsize())
                
                if(canPopfromQ):
                    #print('Popping from Queue ...')
                    inObj = globalQ.get()
                    sz = len(inObj)
                    if(canPrintLen and (sz == self.bufSize)):
                        #print(inObj[:36]) #got header
                        isHdrPkt = True
                        # cnt += 1
                    
                    if((sz < self.bufSize) and not isHdrPkt):
                        canPrintLen = True
                        # cnt += 1
                        # print('Total Data Count: ', cnt, ' -->Last buffer len: ', sz)
                    #     cnt = 0
                    #     res = self.protocol.getMatch(inObj)
                    #     if(res):
                    #         print('\n---------------------LAST BUFF READ------------------------------')
                    #         print(res)
                    #         print('Irregularity caught!!!!')
                    #         exit(-99)
                    
                    # if((sz == self.bufSize) and not isHdrPkt):
                    #     cnt += 1
                    #     continue
                    
                    if(xsData):
                        inObj = xsData + inObj
                        data = io.BytesIO(inObj)
                        xsData = bytes()
                        sz = len(inObj)
                        print('Prepended xsData ...')
                    else:
                        data:io.BytesIO = io.BytesIO(inObj)
                
                    if(contRead):#if fetching subsequent bytes
                        #print('Fetching subsequent for ObjectType: ', curId)
                        rd = data.read(remainingLen)
                        #print('Subsequent:  ', rd)
                        curBf += rd
                        if(len(rd) == remainingLen):
                            #print('Appending to corresponding deque ...')
                            if(curId == DataObjects.Ids.AcousticVideoData.value):
                                if(acVidQ is not None):
                                    acVidQ.put(io.BytesIO(curBf)) #push on queue
                            elif(curId == DataObjects.Ids.VideoData.value):
                                if(vidQ is not None):
                                    vidQ.put(io.BytesIO(curBf))
                            elif(curId == DataObjects.Ids.SpectrumData.value):
                                if(specQ is not None):
                                    specQ.put(io.BytesIO(curBf))
                            elif(curId == DataObjects.Ids.AudioData.value):
                                if(audQ is not None):
                                    audQ.put(io.BytesIO(curBf))
                            elif(curId == DataObjects.Ids.ThermalVideoData.value):
                                if(thermalQ is not None):
                                    thermalQ.put(io.BytesIO(curBf))
                            elif(curId == DataObjects.Ids.RawData.value):
                                if(rawQ is not None):
                                    rawQ.put(io.BytesIO(curBf))
                            elif(curId == DataObjects.Ids.CommonStatus.value):
                                #TODO: update camera status here
                                pass

                            #reset params
                            curId = 0
                            curBf = bytes()
                            contRead = False 
                            remainingLen = 0
                            if(remainingLen == sz):
                                continue

                            #reset buffers
                            inObj = data.read(-1) # read all
                            data = io.BytesIO(inObj)

                        else: #if not all bytes fetched, recalculate remaining length
                            remainingLen = remainingLen - len(rd)
                            #print('More data required. RemainingLength: ', remainingLen)
                            continue #go back to start
                else:
                    #print('previous Queue processing ...')
                    #reset buffers
                    inObj = data.read(-1)
                    data = io.BytesIO(inObj)
                    canPrintLen = True
                    isHdrPkt = True
                    #print('\nPrev Content: ', inObj)
                
                if(canPrintLen and isHdrPkt):
                    if(inObj):
                        hdr = data.read(12)
                        #print(hdr)
                        dmsg = protocol.unpackDataMessage(hdr)
                        #print(dmsg)
                    else:
                        #print('msg failure! -> empty buffer')
                        canPopfromQ = True
                        continue

                    #print('Pre-dmsg (if)', dmsg)
                    if((dmsg.Command == DataMessages.CommandCodes.DataMessage.value) and 
                       (dmsg.ObjectCount <= 10)):
                        # dt = data.read(8)
                        # dobj:DataObjects.DataObjHeader = protocol.unpackDataObjectHeader(dt)
                        # if(dobj.Id != DataObjects.Ids.VideoData.value):
                        #     print('Object count: %i | Datamessage Length: %i' % (dmsg.ObjectCount, dmsg.DataLength + 8))
                        #     print(dobj)
                        # canPrintLen = False
                        # isHdrPkt = False
                        # canPopfromQ = True
                        # continue
                        for i in range(dmsg.ObjectCount):
                            dt = data.read(8)
                            dobj:DataObjects.DataObjHeader = DataObjects.DataObjHeader._make(
                                protocol.unpackDataObjectHeader(dt))
                            if(dobj is None):
                                print('object header returned None!')
                                time.sleep(3.0)
                                exit(-99)
                            # else:
                            #     if(dobj.Id > 50):
                            #         print('\Message header: ', dt)
                            #         print('Processing DataObject| Id: %i, Length: %i' % (dobj.Id, dobj.Length))
                            #         exit(-99)
                                    
                            
                            if(dobj.Id == DataObjects.Ids.AcousticVideoData.value):
                                #print('Got Acoustic-Video data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(acVidQ is not None):
                                        acVidQ.put(io.BytesIO(curBf)) #push on q
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('AcVid: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.VideoData.value):
                                #print('Got Video data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(vidQ is not None):
                                        vidQ.put(io.BytesIO(curBf))
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('Vid: subsequent triggered. remaining: ', remainingLen)
                                    break
                            
                            elif(dobj.Id == DataObjects.Ids.SpectrumData.value):
                                #print('Got Spectrum data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(specQ is not None):
                                        specQ.put(io.BytesIO(curBf)) #push on deque
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('Spec: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.AudioData.value):
                                #print('Got Audio data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(audQ is not None):
                                        audQ.put(io.BytesIO(curBf)) #push on deque
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('Audio: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.RawData.value):
                                #print('Got Raw data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(rawQ is not None):
                                        rawQ.put(io.BytesIO(curBf)) #push on deque
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('Raw: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.ThermalVideoData.value):
                                #print('Got Thermal-Video data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length):
                                    if(thermalQ is not None):
                                        thermalQ.put(io.BytesIO(curBf)) #push on deque
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('Thermal: subsequent triggered. remaining: ', remainingLen)
                                    break
                            
                            elif(dobj.Id == DataObjects.Ids.CommonStatus.value):
                                print('Got CommonStatus data')
                                curBf = data.read(dobj.Length)
                                if(len(curBf) == dobj.Length): #TODO: update camera status
                                    print(data.read(dobj.Length))
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - len(curBf)
                                    #print('CommonStatus: subsequent triggered. remaining: ', remainingLen)
                                    break

                            #check if inMsg still has stuff that can be read
                            curIdx = data.tell()
                            if(data.read(-1)):
                                data.seek(curIdx) #restore
                                canPopfromQ = False
                            else: #proceed normally
                                canPopfromQ = True
                        
                        #print('For loop break')
                        canPrintLen = False
                        isHdrPkt = False
                        canPopfromQ = True
                    else:
                        #print('\nPacket discarded! - Size: ', len(inObj))
                        # print('Previous data (< 1460): \n', store)
                        # store = b''
                        # print('Previous data (== 1460): \n', store2)
                        # store2 = b''
                        #print(inObj[:128])
                        res = self.getMatch(inObj)
                        if(res):
                            idx = res.span()[0] - 12
                            #print(res, ' Hdr at: ', idx)
                            data = io.BytesIO(inObj[idx: ])
                            canPopfromQ = False
                        else:
                            canPopfromQ = True
                         

    '''Decodes and Publishes Video data'''
    def processVideo(self, q:Queue, canRun:multiprocessing.Event):
        print('Starting video processor ...')

        bwvdQ:Queue = self.qDict[SoundCamConnector.DTypes.O_BW.name]
        ovdQ:Queue = self.qDict[SoundCamConnector.DTypes.O_VID.name]
        acQ:Queue = None
        # spcQ: Queue = None
        # audQ: Queue = None
        # thmQ: Queue = None

        if(self.overlayBA[SoundCamConnector.OverlayTypes.ACOUSTIC.value]):
            acQ = self.qDict[SoundCamConnector.DTypes.O_ACVID.name]
        # if(self.overlayBA[SoundCamConnector.OverlayTypes.SPECTRUM.value]):
        #     spcQ = self.qDict[SoundCamConnector.DTypes.O_SPEC.name]
        # if(self.overlayBA[SoundCamConnector.OverlayTypes.AUDIO.value]):
        #     audQ = self.qDict[SoundCamConnector.DTypes.O_AUD.name]
        # if(self.overlayBA[SoundCamConnector.OverlayTypes.THERMAL.value]):
        #     thmQ = self.qDict[SoundCamConnector.DTypes.O_THM.name]


        # vidOut = cv2.VideoWriter('soundcam.mp4', cv2.VideoWriter_fourcc(*'AVC1'), 17, 
        #                             (self.cfgObj['camera_H'], self.cfgObj['camera_W']))
        #vidOut = cv2.VideoWriter('soundcam.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (240, 320))
        start_t = time.time()
        hits = 0
        acFrame = None
        while(canRun.is_set()):
            if(not q.empty()):
                #print('Video Q size: ', q.qsize())
                decoded = self.protocol.unpackDecodeVideoData(q.get())
                if(self.cfgObj['visualizeVideo']):
                    if(self.cfgObj['visualizeOverlay'] and not acQ.empty()):
                        acFrame = acQ.get()
                    self.visualizeBWVideo(decoded[1], acFrame=acFrame)
                else:
                    if(not acQ.empty()):
                        acFrame = acQ.get()

                    ovid = self.visualizeBWVideo(decoded[1], acFrame=acFrame, getResult=True)
                    if(not ovdQ.full()): #push data to Q
                        ovdQ.put(ovid)
                    else:
                        try:
                            ovdQ.get_nowait() # remove data
                        except Exception as ex:
                            pass
                        ovdQ.put(ovid)
                    
                    if(not bwvdQ.full()): #push data to Q
                        bwvdQ.put(decoded[1])
                    else:
                        try:
                            bwvdQ.get_nowait() # remove data
                        except Exception as ex:
                            pass
                        bwvdQ.put(decoded[1])
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('==================================================================Vid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()

    '''Decodes and Publishes Acoustic Video data'''
    def processAcVideo(self, q:Queue, canRun:multiprocessing.Event, scam:SU, 
                       semaph:multiprocessing.Semaphore):
        print('Starting acoustic video processor ...')
        acQ:Queue = self.qDict[SoundCamConnector.DTypes.O_ACVID.name]
        start_t = time.time()
        hits = 0
        while(canRun.is_set()):
            if(not q.empty()):
                decoded = self.protocol.unpackDecodeAcousticVideoData(q.get())
                with semaph:
                    norm0_1 = scam.updateAcousticBuffer(decoded)
                    if(self.cfgObj['visualizeAcVideo']):
                        self.visualizeACVideo(norm0_1)

                    if(not acQ.full()): #push acoustic data to overlay_Q
                        acQ.put(self.visualizeACVideo(norm0_1, getResult=True))
                    else:
                        try:
                            acQ.get_nowait() # remove data
                        except Exception as ex:
                            pass
                        acQ.put(self.visualizeACVideo(norm0_1, getResult=True))
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('===============================================================AcousticVid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()

    '''Decodes and Publishes Spectrum data'''
    def processSpectrum(self, q:Queue, canRun:multiprocessing.Event, scamObj:SU, 
                        semaph:multiprocessing.Semaphore):
        print('Starting spectrum processor ...')

        saveSpectrum = False
        specData = []
        t = 30 #seconds
        record_t = time.time()
        spcQ: Queue = self.qDict[SoundCamConnector.DTypes.O_SPEC.name]
        hits = 0
        start_t = time.time()

        #detection algorithm
        detect_start = time.time()
        has_started = False
        while(canRun.is_set()):
            if(not q.empty()):
                decoded = self.protocol.unpackDecodeSpectrumData(q.get())
                with semaph:
                    scamObj.updateSpectrumBuffer(decoded)
                    energy_info = self.scamUtils.computeEnergy(getArray=False)
                    if(not has_started):
                        if((energy_info[0] > self.cfgObj['mean_energy_thresh']) and 
                        (energy_info[1] > self.cfgObj['energy_std_dev'])):
                            print("Activation!!!")
                            print(f"Mean Energy: {energy_info[0]:.4f}")
                            print(f"Standard deviation of energy: {energy_info[1]:.4f}")
                            detect_start = time.time()
                            has_started = True
                    else:
                        #print('Will update detection: ', self.scamUtils.p_getDetection())
                        if((time.time() - detect_start >= self.cfgObj['trigger_time'])):
                            self.scamUtils.updateDetection(True)
                        if((energy_info[0] < self.cfgObj['mean_energy_thresh']) and 
                        (energy_info[1] < self.cfgObj['energy_std_dev'])):
                            print("Deactivation!!!")
                            print(f"Mean Energy: {energy_info[0]:.4f}")
                            print(f"Standard deviation of energy: {energy_info[1]:.4f}")
                            has_started = False
                            self.scamUtils.updateDetection(False)
                        
                    
                
                #print('Spectrum:', decoded[0], ' ' ,decoded[1])
                if(spcQ and not spcQ.full()): #push spectrum data to overlay_Q
                    spcQ.put(decoded)
                else:
                    try:
                        spcQ.get_nowait() # remove data
                    except Exception as ex:
                        pass
                    if(spcQ):
                        spcQ.put(decoded)
                if(saveSpectrum):
                    specData.append(decoded[2])
                    if(time.time() - record_t >= t):
                        sr = int(decoded[0][1] * 1024 * 2) #nyquist
                        print('saving spectrum with sampling frequency (Hz): ', sr)
                        # specData = np.concatenate(specData)
                        # self.scamUtils.writeAudio('outputSpec_mute.wav', specData, sr)
                        self.scamUtils.writeOutVar('whitenoise_mat_t2')
                        saveSpectrum = False
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('===============================================================Spectrum @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()

    '''Decodes and Publishes Audio data'''
    def processAudio(self, q:Queue, canRun:multiprocessing.Event, scamObj:SU, 
                     semaph:multiprocessing.Semaphore):
        print('Starting audio processor ...')

        saveAudio = False
        audioData_plus = []
        audioData_minus = []
        t = 30 #seconds
        record_t = time.time()

        audQ:Queue = self.qDict[SoundCamConnector.DTypes.O_AUD.name]
        start_t = time.time()
        hits = 0
        while(canRun.is_set()):
            if(not q.empty()):
                decoded = self.protocol.unpackDecodeAudioData(q.get())
                with semaph:
                    scamObj.updateAudioBuffer(decoded) #update ringbuffer
                if(audQ and not audQ.full()): #push audio data to overlay_Q
                    audQ.put(decoded)
                else:
                    try:
                        audQ.get_nowait() # remove data
                    except Exception as ex:
                        pass
                    if(audQ):
                        audQ.put(decoded)
                if(saveAudio):
                    audioData_plus.append(decoded[2])
                    audioData_minus.append(decoded[3])
                    if(time.time() - record_t >= t):
                        sr = 1/ decoded[1][0]
                        print('\n--------------Saving audio with sampling frequency (Hz): ', sr)
                        audioData_plus = np.concatenate(audioData_plus)
                        audioData_minus = np.concatenate(audioData_minus)
                        #audioData /= np.max(np.abs(audioData),axis=0)
                        #self.scamUtils.writeAudio('outputAudio_single_plus.wav', audioData_plus, sr)
                        #self.scamUtils.writeAudio('outputAudio_single_minus.wav', audioData_minus, sr)
                        scamObj.writeAudio('outputLong.wav', (audioData_plus + audioData_minus), sr)
                        saveAudio = False
                        
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Audio @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()

    '''Decodes and Publishes Raw data'''
    def publishRaw(self, q:Queue, canRun:multiprocessing.Event):
        print('Starting raw data publisher (for debugging) ...')
        while(canRun.is_set()):
            pass

    '''Decodes and Publishes Thermal Video data'''
    def processThermalVideo(self, q:Queue, canRun:multiprocessing.Event):
        print('Starting thermal video processor ...')
        tmQ:Queue = self.qDict[SoundCamConnector.DTypes.O_THM.name]
        start_t = time.time()
        hits = 0
        while(canRun.is_set()):
            if(not q.empty()):
                decoded = self.protocol.unpackDecodeThermalVideoData(q.get())
                if(self.cfgObj['visualizeThermal']):
                    self.visualizeThermal(decoded[1])
                else:
                    tmvid = self.visualizeThermal(decoded[1], getResult=True)
                    if(not tmQ.full()): #push data to Q
                        tmQ.put(tmvid)
                    else:
                        try:
                            tmQ.get_nowait() # remove data
                        except Exception as ex:
                            pass
                        tmQ.put(tmvid)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Thermal @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()

    '''Updates the plot'''
    def plotUpdater(self, frame):
        if(self.cfgObj['visualizeAudio']):
            if(not self.audQ_RT.empty()):
                data = self.audQ_RT.get()
                audio = data[2] + data[3]
                lims = np.max(audio) + 20000 # plus buffer range
                self.ax[0].set_ylim(top=lims, bottom=-lims)
                self.plotLines[0].set_ydata(audio)
        
        if(self.cfgObj['visualizeAcVideo']):
            self.plotLines[3].set_ydata(self.acousticBuffer)
            #print(acEnergy.shape, ' ', acEnergy[-5:])
            #TODO:
            '''
            I suppose that taking the max per bucket & subtracting the (dynamik)
            should give the array with the detected sound (and surroundings) from which
            that background/ mean can be subracted in order to visualize the actual sound.
            The use of the crest value can further be used to raise the threshold
            '''
            self.plotLines[4].set_ydata(np.average(self.acousticBuffer.reshape(-1, 3072), axis=1))
            self.plotLines[5].set_ydata(np.max(self.acousticBuffer.reshape(-1, 3072), axis=1))
            self.plotLines[6].set_ydata(self.energyBuffer)
        
        if(self.cfgObj['visualizeSpectrum']):
            if(not self.spcQ_RT.empty()):
                data = self.spcQ_RT.get()
                self.plotLines[1].set_ydata(data[2]) #global
                self.plotLines[2].set_ydata(data[3]) #local
                
                # spectogram plotting
                self.ax_img = self.scamUtils.updateSpectogramPlot()
            return self.plotLines, self.ax_img
        return self.plotLines
    

    
    '''Displays live AUDIO/SPECTRUM data'''
    def displayLiveData(self, canRun:multiprocessing.Event):
        print('Starting visual display')
        self.spcQ_RT: Queue = self.qDict[SoundCamConnector.DTypes.O_SPEC.name]
        self.audQ_RT: Queue = self.qDict[SoundCamConnector.DTypes.O_AUD.name]
        #wait for data
        while(canRun.is_set()):
            print('Awaiting data ...')
            if(self.scamUtils.canRunAnalysis()):
                print('Ready to run analysis ...')
                break
            else:
                time.sleep(0.5)

        self.ax:list[Axes] = []
        fig:Figure = None
        fig, self.ax = plt.subplots(nrows=6, ncols=1, figsize=(16, 16))
        self.plotLines:list[Line2D] = []
        
        #Audio Plot
        self.ax[0].set_title('Audio Data')
        self.focusedAudio = np.zeros(2048, dtype=np.int32)
        x1 = np.linspace(1, 2048, 2048)
        self.ax[0].set_ylim(top=5000, bottom=-5000)
        self.ax[0].set_facecolor((0.18, 0.176, 0.18))
        audlines, = self.ax[0].plot(x1, self.focusedAudio, 
                                    color=(0.85, 1, 0.0), label='focused audio')
        self.plotLines.append(audlines)

        #Linear Spectrum Plot
        self.ax[1].set_title('Linear Spectrum Plot')
        x2 = np.linspace(0, 100000, 1023)
        self.ax[1].set_ylim(top=self.cfgObj['maximum_spdb'], 
                    bottom=self.cfgObj['minimum_spdb'])  # set safe limit to ensure that all data is visible.
        self.ax[1].set_facecolor((1, 1, 1))
        sline1, = self.ax[1].plot(x2, np.zeros(1023, dtype=np.float32), 
                                  color=(0.0, 0.0, 0.0), label='global spectrum')
        self.plotLines.append(sline1)
        sline2, = self.ax[1].plot(x2, np.zeros(1023, dtype=np.float32), 
                                  color=(0.95, 0.4, 0.1), label='local spectrum')
        self.plotLines.append(sline2)

        #Spectrogram Plot
        if(self.cfgObj['visualizeSpectrum']):
            self.ax[2].set_title('Spectrum Analysis')
            self.ax[2].set_xlabel('Time (seconds)')
            self.ax[2].set_ylabel('Frequency (Hz)')

            self.ax_img = self.scamUtils.initSpectogram(ax=self.ax[2])
            fig.colorbar(self.ax_img, ax=self.ax[2], format="%+2.f dB")
        
        #Acoustic Plot
        x3 = np.linspace(0, 5, self.acousticBuffer.shape[0])
        self.ax[3].set_ylim(top=self.cfgObj['maximum_db'], 
                            bottom=self.cfgObj['minimum_db'])
        #self.ax[3].set_facecolor((0.18, 0.176, 0.18))
        acsLines, = self.ax[3].plot(x3, np.zeros(self.acousticBuffer.shape[0], dtype=np.float32), 
                                    color=(0.176, 0.482, 0.91), label='acoustic')
        self.plotLines.append(acsLines)

        #Acoustic Energy Plot
        x4 = np.linspace(0, 5, int(self.acousticBuffer.shape[0]/3072))
        self.ax[4].set_ylim(top=self.cfgObj['maximum_db'], 
                            bottom=self.cfgObj['minimum_db'])
        acsEnergyLines, = self.ax[4].plot(x4, np.zeros(int(self.acousticBuffer.shape[0]/3072), dtype=np.float32), 
                                    color=(1, 0.043, 0.173), label='acoustic-energy')
        self.plotLines.append(acsEnergyLines)
        acsMax, = self.ax[4].plot(x4, np.zeros(int(self.acousticBuffer.shape[0]/3072), dtype=np.float32), 
                                    color=(1, 0, 0.992), label='acoustic-max')
        self.plotLines.append(acsMax)


        #Energy Plot
        x5 = np.linspace(0, 5, self.energyBuffer.shape[0])
        self.ax[5].set_ylim(top=100, bottom=0)  # set safe limit to ensure that all data is visible.
        self.ax[5].set_facecolor((1, 1, 1))
        energyLines, = self.ax[5].plot(x5, np.zeros(self.energyBuffer.shape[0], dtype=np.float32), 
                                  color=(0.263, 0.839, 0.118), label='energy')
        self.plotLines.append(energyLines)

        fig.legend()
        fig.tight_layout()

        self.visualUp = True #for logic in startStream
        ani = FuncAnimation(fig, self.plotUpdater, interval=1, blit=False, cache_frame_data=False)
        # Writer = writers['ffmpeg']
        # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800) 
        # ffmpegWritter = FFMpegWriter(fps=15)      
        
        try:
            print('Showing figure')
            plt.show()
            print('Figure closed')
            # ani.save('stream.mov', writer=ffmpegWritter)
        except Exception as ex:
            print(ex)

        while(canRun.is_set()):
            pass
        plt.close()

    def displayProcess(self, canRun:multiprocessing.Event):
        print('Starting display process')
        self.displayLiveData(canRun=canRun) #display audio/spectrum signal(s)

    ''' Post Processes data for comprehensive visualization '''
    def postProc(self):
        print('Starting post-processing process')
        while(self.processData.is_set()):
            try:
                print('post processing')
                vimg = self.vid_recv.recv()
                acimg = self.acvid_recv.recv()
                if((acimg is not None) and (vimg is not None)):
                    print('Got AcImg w Size: ', acimg.size)
                    print('Got VidImg w Size: ', vimg.size)
                    alpha = 0.7
                    beta = (1.0 - alpha)
                    dst = cv2.addWeighted(vimg, alpha, acimg, beta, 0.0)
                    cv2.imshow('merge', dst)
                    if cv2.waitKey(1) == ord('q'):
                        exit(-99)
            except Exception as ex:
                print(ex)
    
    '''
    ----------------------------------------------------------------------DATA ACCESS METHODS
    '''

    def hasStatus(self):
        return self.protocol.p_hasInitialStatus()
    '''
        Returns camera status
            - deviceId
            - protocolVersion
            - hardwareVersion
            - firmwareVersion
            - deviceError
            - cpuTemperature
            - temperatureTop
            - temperatureBottom
            - deviceState
            - subState
            - deviceIP
            - features
    '''
    def getStatus(self): 
        return self.protocol.getDeviceStatus()
    
    ''' Return connection status '''
    def isConnected(self):
        return self.connected

    ''' Returns the BW Video frame '''
    def getBWVideo(self):
        if(not self.qDict[SoundCamConnector.DTypes.O_BW.name].empty()):
            return self.qDict[SoundCamConnector.DTypes.O_BW.name].get()
        return None
    
    ''' Returns the Overlayed video frame '''
    def getOverlayVideo(self):
        if(not self.qDict[SoundCamConnector.DTypes.O_VID.name].empty()):
            return self.qDict[SoundCamConnector.DTypes.O_VID.name].get()
        return None
    
    ''' Returns the Acoustic Video frame '''
    def getACVideo(self):
        if(not self.qDict[SoundCamConnector.DTypes.O_ACVID.name].empty()):
            return self.qDict[SoundCamConnector.DTypes.O_ACVID.name].get()
        return None
    
    ''' Returns the Thermal Video frame '''
    def getTMVideo(self):
        if(not self.qDict[SoundCamConnector.DTypes.O_THM.name].empty()):
            return self.qDict[SoundCamConnector.DTypes.O_THM.name].get()
        return None
    
    ''' Returns the Audio frame '''
    def getAudio(self):
        with self.aud_semaphore:
            return self.scamUtils.getAudioBuffer()[-2048:]
        
    ''' Returns the Audio Info '''
    def getAudioInfo(self):
        if(not self.qDict[SoundCamConnector.DTypes.O_AUD.name].empty()):
            decoded = self.qDict[SoundCamConnector.DTypes.O_AUD.name].get()
            meta:DataObjects.MDAudioData2 = DataObjects.MDAudioData2._make(decoded[1])
            srate = int(1/meta.dt)
            return dict({'channels': 1,
                        'sample_rate': srate,
                        'bitrate': 16 * srate})
        return None
    
    ''' Returns the Spectrum frame '''
    def getSpectrum(self):
        with self.spec_semaphore:
            return (self.scamUtils.p_getFrequencies(), self.scamUtils.getSpectrumBuffer()[-1023:])
    
    ''' Returns the current blob data '''
    def getBlobData(self):
        return self.scamUtils.p_getBlobData()
    
    ''' Returns the Mean Energy and Std. Dev in the Spectrum '''
    def getEnergy(self):
        return self.scamUtils.computeEnergy(getArray=False)
    
    ''' Returns the detection status '''
    def hasDetection(self):
        return self.scamUtils.p_getDetection()
    
    ''' Sets & Returns the scaling Mode for the Acoustic filter '''
    def setScalingMode(self, mode, max=None, dynamic=5.0, crest=3.1):
        #configure acoustic filter
        self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                    max=max, crest=crest)
        return True
    
    def getScalingMode(self):
        return self.scamUtils.getScalingMode()

    '''
    --------------------------------------------------------------------VISUALIZATION METHODS
    '''

    '''Visualizes Thermal Video'''
    def visualizeThermal(self, frame, getResult=False):
        if(frame is None):
            return None
        frame = np.interp(frame, (-10, 50), (0, 255))
        frame = frame.astype('uint8')
        mimg = np.zeros((120, 160, 3), dtype=np.uint8)
        cv2.applyColorMap(frame, cv2.COLORMAP_INFERNO, mimg)
        if(getResult):
            return mimg
        cv2.namedWindow("Thermal Video", cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Thermal Video', mimg)
        if cv2.waitKey(1) == ord('q'):
            exit(-99)

    '''Visualizes Acoustic Video'''
    def visualizeACVideo(self, frame, getResult=False):
        truth_tbl = np.where(frame[:,:] == 0.0, True, False)
        frame *= 255
        frame = frame.astype('uint8')
        mimg = np.zeros((48, 64, 3), dtype=np.uint8)
        cv2.applyColorMap(frame, cv2.COLORMAP_JET, mimg)
        mimga = cv2.cvtColor(mimg, cv2.COLOR_RGB2RGBA)
        mimga[truth_tbl, 3] = 0

        #kernel = np.ones((8,8),np.float32)/64
        #filt_img = cv2.filter2D(mimga,-1,kernel)
        #filt_img = cv2.GaussianBlur(mimga, (5,5), 0)
        rimg_up = cv2.resize(mimga, 
                                (self.cfgObj['camera_W'], self.cfgObj['camera_H']), 
                                interpolation= cv2.INTER_AREA)
        boxes_ids:list = self.objTracker.detection(rimg_up, track=False)
        blobData = []
        if(boxes_ids is not None):
            if(len(boxes_ids) == 0):
                self.objTracker.resetIdCount()
            else:
                blobData = boxes_ids[0]
                #print("Detection: {0}".format(boxes_ids))
                for box_id in boxes_ids:
                    x,y,w,h,A = box_id
                    if(A > blobData[4]):
                        blobData = box_id
            # print("Detection x-y: {0}".format(blobData))
            # print("Has Positive Detection? {0}".format(self.hasDetection()))
        if(getResult):
            #update blobData
            self.scamUtils.updateBlobData(blobData)
            return rimg_up
        cv2.namedWindow("Acoustic Video", cv2.WINDOW_AUTOSIZE)
        #bimg_up = cv2.GaussianBlur(rimg_up, (5,5), 0)
        cv2.imshow('Acoustic Video', rimg_up)
        if cv2.waitKey(1) == ord('q'):
            exit(-99)
    
    ''' Visualizes Black/White Video [with Overlay] '''
    def visualizeBWVideo(self, frame, acFrame:np.array=None, getResult=False):
        bw_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGRA)
        if(self.debug):
            font = cv2.FONT_HERSHEY_SIMPLEX
            self.new_frame_t = time.time()
            fps = 1/(self.new_frame_t - self.prev_frame_t)
            self.prev_frame_t = self.new_frame_t
            fps =  int(fps)
            cv2.putText(bw_img, str(fps), (7, 70), font, 2, (100, 255, 0), 3, cv2.LINE_AA)
        if(self.cfgObj['visualizeOverlay']):
            if(acFrame is not None): # overlay acoustic on video image
                #print(acFrame.shape)
                # store the alpha channels only
                m1 = bw_img[:,:,3]
                m2 = acFrame[:,:,3]

                # invert the alpha channel and obtain 3-channel mask of float data type
                m1i = cv2.bitwise_not(m1)
                alpha1i = cv2.cvtColor(m1i, cv2.COLOR_GRAY2BGRA)/255.0

                m2i = cv2.bitwise_not(m2)
                alpha2i = cv2.cvtColor(m2i, cv2.COLOR_GRAY2BGRA)/255.0

                # Perform blending and limit pixel values to 0-255 (convert to 8-bit)
                b1i = cv2.convertScaleAbs(acFrame*(1-alpha2i) + bw_img*alpha2i)
                if(getResult):
                    return b1i
                cv2.imshow('Video - Overlay', b1i)
        else:
            if(getResult):
                return bw_img
            cv2.imshow('Video Data', bw_img)
        if cv2.waitKey(1) == ord('q'):
            exit(-99)
        

if __name__ == '__main__':
    from config import cfgContext
    camObj = SoundCamConnector(debug=True, cfgObj=cfgContext)

    atexit.register(camObj.release_shared)
    atexit.register(camObj.disconnect)

    if(camObj.connect()):
        print('Sending Initial configuration and preparing state')
        camObj.initialConfigure()
        while(not camObj.hasStatus()):
            pass
        print(camObj.getStatus(), '\n')

    #camObj.restartCamera()
    #time.sleep(3.0)
    camObj.startMeasurement()
    while(camObj.processData):
        time.sleep(0.001)