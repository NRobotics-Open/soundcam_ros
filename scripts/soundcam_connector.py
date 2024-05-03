#!/usr/bin/env python3

'''
This package is developed by N Robotics GmbH to interface with the Sound Camera 
from CAE Sofware & Systems GmbH.

Developer: ephson@nrobotics.com
'''

import socket, time, atexit, struct, os, io, re, sys
import asyncio
from asyncio import Transport
from threading import Thread
from enum import Enum
import numpy as np, cv2
np.set_printoptions(threshold=sys.maxsize)

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from soundcam_protocol import CommandCodes, DataMessages, Device, DataObjects, CameraProtocol
from multiprocessing import Process, Queue, Pipe
from multiprocessing.connection import Connection
from soundcam_ros import SoundcamROS
from soundcam_async_client import SoundcamClientProtocol

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

    def __init__(self, deviceIP:str='169.254.169.138', devicePort:int=6340, protocol:int=3, debug:bool = False) -> None:
        print('Firmware Version: ', FIRMWARE_VERSION)
        self.debug = debug
        self.testing = False
        self.invokeId = 1
        self.pubObj = SoundcamROS()

        #check SoundCam Connectivity and Initialize connection
        if(not self.pingCamera(deviceIP=self.pubObj.cfg['ip'])):
            exit(-99)

        self.bufSize = 1460
        self.recvStream = False
        self.processData = True
        self.visualUp = False
        self.isConnected = False

        #prepare queues
        self.processes = list()
        self.qDict = dict()
        
        if(self.pubObj.cfg['publish_video']):
            self.vid_recv, vid_send = Pipe()
            self.vidQ = Queue()
            self.qDict[SoundCamConnector.DTypes.VID.name] = self.vidQ
            self.processes.append(Process(target=self.publishVideo, args=(self.vidQ, self.processData, vid_send)))
        if(self.pubObj.cfg['publish_acoustic_video']):
            self.acvid_recv, acvid_send = Pipe()
            self.acVidQ = Queue()
            self.qDict[SoundCamConnector.DTypes.ACOUSTIC_VID.name] = self.acVidQ
            self.processes.append(Process(target=self.publishAcousticVideo, 
                                          args=(self.acVidQ, self.processData, acvid_send)))
        if(self.pubObj.cfg['publish_spectrum']):
            self.spec_recv, spec_send = Pipe()
            self.specQ = Queue()
            self.qDict[SoundCamConnector.DTypes.SPECTRUM.name] = self.specQ
            self.processes.append(Process(target=self.publishSpectrum, args=(self.specQ, self.processData,
                                                                             spec_send)))
        if(self.pubObj.cfg['publish_audio']):
            self.aud_recv, aud_send = Pipe()
            self.audQ = Queue()
            self.qDict[SoundCamConnector.DTypes.AUDIO.name] = self.audQ
            self.processes.append(Process(target=self.publishAudio, args=(self.audQ, self.processData, 
                                                                          aud_send)))
        if(self.pubObj.cfg['publish_thermal']):
            self.thermalQ = Queue()
            self.qDict[SoundCamConnector.DTypes.THERMAL.name] = self.thermalQ
            self.processes.append(Process(target=self.publishThermalVideo, 
                                          args=(self.thermalQ, self.processData)))
        if(self.pubObj.p_pubRaw):
            self.rawQ = Queue()
            self.qDict[SoundCamConnector.DTypes.RAW.name] = self.rawQ
            self.processes.append(Process(target=self.publishRaw, args=(self.rawQ, self.processData)))
        
        if(self.pubObj.cfg['visualize_audio'] or self.pubObj.cfg['visualize_spectrum']):
            self.processes.append(Process(target=self.displayProcess))
        
        if(self.pubObj.cfg['publish_video_acoustic_overlay']):
            self.processes.append(Process(target=self.postProc))

        #camera protocol instance
        self.protocol = CameraProtocol(protocol=self.pubObj.cfg['protocol'], debug=self.debug) #holds & processes received data from device

        #start processes
        print('Starting processes ...')
        self.globalQ = Queue()
        self.qDict[SoundCamConnector.DTypes.GLOBAL.name] = self.globalQ 
        self.processes.append(
            Process(target=self.streamFilter, args=(self.qDict, self.processData, self.protocol)))
        
        for proc in self.processes:
            proc.start()
        
        #start cyclic thread
        th = Thread(target=self.receiveCyclic, daemon=True)
        th.start()

        #Note: All data is sent as Little Endian!
    
    '''Ping Camera'''
    def pingCamera(self, deviceIP):
        #check SoundCam Connectivity and Initialize connection
        result = False
        start_t = time.time()
        while(True):
            res = os.system("ping -c 1 " + deviceIP)
            if(res == 0):
                result = True
                break
            if((time.time() - start_t) > CONNECT_TIMEOUT):
                print('Camera not connected. Check network configuration!')
                break
            time.sleep(0.5)
        return result

    '''
        Establishes connection with the camera and performs handshake (gets device Id)
    '''
    def connect(self):
        ip_addr = self.pubObj.cfg['ip'] 
        port = self.pubObj.cfg['port']
        self.sock = socket.socket(socket.AF_INET, # IPv4
                        socket.SOCK_STREAM) # UDP SOCK_DGRAM   #TCP SOCK_STREAM
        if hasattr(socket, "SO_REUSEPORT"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        try:
            self.sock.connect((ip_addr, port))
            self.isConnected = True
        except Exception as ex:
            print('Failure Establishing connection to device! ', ex)
            self.isConnected = False
            exit(-1)
        print("\n Socket initialized on: \t IP| %s PORT| %d" % (ip_addr, port))
        #Request device Identification
        query = self.protocol.generateReadRequest(command=CommandCodes.IdentificationReq, 
                                                  invokeId=self.invokeId)
        self.invokeId += 1
        query += self.protocol.writeCameraLighting(self.invokeId, 
                                                   self.pubObj.cfg['cameraLighting'])
        self.sendData(query=query)
        return self.isConnected

    '''Sends command to reset the camera'''
    def restartCamera(self):
        query = self.protocol.generateReadRequest(command=CommandCodes.ResetReq, 
                                                  invokeId=self.invokeId)
        self.sendData(query=query)
        
    '''Starts the cyclic reception of stream'''
    def startStream(self):
        print('Stream started/resumed ...')
        self.recvStream = True
        self._startMeasurement()

    
    '''Stops the cyclic reception of stream'''
    def stopStream(self):
        query = self.protocol.setState(invokeId=self.invokeId)
        self.sendData(query=query)
        print('Sent FinishRequest ...')
        self.recvStream = False

    def disconnect(self):
        if(self.isConnected):
            try:
                plt.close()
                #self.stopStream()
                self._stopMeasurement()
                self.sock.close()
            except Exception as ex:
                print('Error closing connection: ', ex)
        self.processData = False
        for proc in self.processes:
            proc.join()

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
    def configure(self, getQuery=False)->bool:
        try:
            #Configure with WriteObjectReq request --- multiple
            query = self.protocol.InitialConfig(self.invokeId, 
                            self.pubObj.cfg['distance'], 
                            (self.pubObj.cfg['minFrequency'], self.pubObj.cfg['maxFrequency']), 
                            (self.pubObj.cfg['camera_W'], self.pubObj.cfg['camera_H']),
                            self.pubObj.cfg['acimageFPS'], 
                            self.pubObj.cfg['acAvg'])
            #concatenate PrepareState
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, 
                                            Device.DeviceStates(self.pubObj.cfg['deviceState']))#set device state
            #concatenate DataToSend
            self.invokeId += 1
            query += self.protocol.dataToSendConfig(self.invokeId, 
                                                    self.pubObj.cfg['dataToSend'])
            if(getQuery):
                return query
            self.sendData(query=query)
            #self._startMeasurement()
        except Exception as ex:
            print('Error Configuring device!', ex)
            return False
        return True
    
    def _startMeasurement(self)->bool:
        try:
            """ query = self.protocol.setState(self.invokeId, Device.DeviceStates.Streaming)#set streaming state
            self.sendData(query=query)
            query = self.protocol.dataToSendConfig(self.invokeId)
            self.sendData(query=query) """
            print('Starting measurement ...')
            query = self.protocol.startStopProcedure(self.invokeId) #start procedure
            self.sendData(query=query)
            self.recvStream = True
        except Exception as ex:
            print('Error Starting Measurement!')
            return False
        self.protocol.setStreamFlags()
        return True
    
    def _stopMeasurement(self)->bool:
        try:
            print('Stopping measurement ...')
            query = self.protocol.startStopProcedure(self.invokeId, False) #stop procedure
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, Device.DeviceStates.Idle) #set Idle state
            self.sendData(query=query)
            self.protocol.unsetStreamFlags()
        except Exception as ex:
            print('Error Stopping Measurement')
            return False
        return True
    
    '''Packs the continuous incoming stream to the generic Queue'''
    def receiveCyclic(self):
        print('Cyclic thread started ...')
        start_t = time.time()
        heartrate = 1/ self.pubObj.cfg['heartbeatRate']
        heart_t = time.time()
        commonstatus_t = time.time()
        canPrintLen = True
        while(self.processData):
            if(self.isConnected):
                #print('Receiving ...')
                res = self.sock.recv(self.bufSize)
                if(res):
                    if(self.recvStream):
                        sz = len(res)
                        if(canPrintLen and (sz == 1460)):
                            print(res[:36])
                            canPrintLen = False
                        if((sz < 1460) and not canPrintLen):
                            canPrintLen = True
                        #print('Incoming with length: ', len(res))
                        #self.globalQ.put(res)
                    else: #if not streaming
                        try:
                            """ if((self.protocol.DeviceState == 
                                Device.DeviceStates(self.pubObj.cfg['deviceState'])) and 
                                (self.protocol.SubState == Device.SubStates.Preparing)):
                                #self.recvStream = True #ready for continuous stream
                                #print('Starting measurement')
                                #self._startMeasurement()
                                print('Preparing detected!')
                                time.sleep(10.0) """
                            self.protocol.unpackDecodeResponse(response=res)
                        except Exception as ex:
                            print('\nError Unpacking and Decoding ...')
                            print(ex)
                            print('Length: ', len(res), ' ', res)

                now = time.time()
                if((not self.recvStream) and self.protocol.isConfigured): #query for common status when not receiving stream
                    if((now - commonstatus_t) >= 0.5):
                        query = self.protocol.generateReadRequest(command=CommandCodes.ReadDataObjectReq, invokeId=self.invokeId, dataLs=[DataObjects.Ids.CommonStatus])
                        self.sendData(query)
                        commonstatus_t = now
                
                if((now - start_t) >= 3 and self.isConnected):
                    print('Cyclic idling ...')
                    start_t = time.time()

                """ if((now - heart_t) >= heartrate):
                    #send ACK to device
                    heart_t = now """
                time.sleep(0.0001)
            else:
                print('Socket not connected!')
                time.sleep(0.1)
        
    '''Filters the generic Deque to the corresponding Deques based on the type'''
    def streamFilter(self, qDict:dict, canRun:bool, protocol:CameraProtocol):
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

            
        
        dmsg:DataMessages.MDDataMessage = None
        dmsgIdx = None
        while(canRun):
            #read queue and filter
            if(not globalQ.empty()):
                if(globalQ.qsize() > 5):
                    print('Global Queue Size: ', globalQ.qsize())
                
                if(canPopfromQ):
                    print('Popping from Queue ...')
                    inObj = globalQ.get()
                    
                    if(xsData):
                        inObj = xsData + inObj
                        data = io.BytesIO(inObj)
                        xsData = bytes()
                        #print('Prepended xsData ...')
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
                            if(remainingLen == len(inObj)):
                                continue
                            
                            #cIdx = data.tell()
                            inObj = data.read(-1) # read all
                            data = io.BytesIO(inObj)
                            #data.seek(cIdx) #restore io
                        else: #if not all bytes fetched, recalculate remaining length
                            remainingLen = remainingLen - len(rd)
                            #print('More data required. RemainingLength: ', remainingLen)
                            continue #go back to start
                else:
                    #print('previous Queue processing ...')
                    #cIdx = data.tell()
                    inObj = data.read(-1)
                    data = io.BytesIO(inObj)
                    #data.seek(cIdx) #restore

                #check if writereqres
                """ if((inObj[:1] == CommandCodes.WriteDataObjectRes.value) or 
                   (inObj[:1] == CommandCodes.PrepareStateRes)):
                    print('Detected write resp:', inObj, ' truncating ...' )
                    inObj = b''
                    time.sleep(5.0) """
                
                if(inObj):
                    dmsgIdx = protocol.getMatch(inObj)
                    if(dmsgIdx):
                        #print(dmsgIdx)
                        hdr = inObj[dmsgIdx.span()[0] - 12:dmsgIdx.span()[0]]
                        data.seek(dmsgIdx.span()[0])
                        #hdr = data.read(12)
                        #print(hdr)
                        dmsg = protocol.unpackDataMessage(hdr)
                        if(dmsg is None):
                            #print('Dmsg decoding returned NONE!!')
                            continue
                        #print(dmsg)
                        #continue
                    else:
                        if(inObj):
                            print('Data BUT NO HEADER FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                            print('No header in: \n', inObj)
                            time.sleep(0.5)
                            #exit(-99)
                        
                        canPopfromQ = True
                        continue
                else:
                    canPopfromQ = True
                    continue
                
                #debugging
                #print('Generic buffer length: ', len(inObj))
                
                if(len(inObj) < 32): #if less than CommonStatus size
                    xsData = inObj
                    print('Low Buffer Length: ', len(xsData))
                    canPopfromQ = True
                    continue

                #print('Pre-dmsg (if)', dmsg)
                if(dmsg.Command == DataMessages.CommandCodes.DataMessage.value):
                    #print('Object count: %i | Datamessage Length: %i' % (dmsg.ObjectCount, dmsg.DataLength))
                    for i in range(dmsg.ObjectCount):
                        dt = data.read(8)
                        dobj:DataObjects.DataObjHeader = protocol.unpackDataObjectHeader(dt)
                        if(dobj is None):
                            print('object header returned None!')
                            time.sleep(3.0)
                            exit(-99)
                        else:
                            if(dobj.Id > 50):
                                print('\Message header: ', dt)
                                print('Processing DataObject| Id: %i, Length: %i' % (dobj.Id, dobj.Length))
                                exit(-99)
                                
                        
                        if(dobj.Id == DataObjects.Ids.AcousticVideoData.value):
                            print('Got Acoustic-Video data')
                            curBf = data.read(dobj.Length)
                            if(len(curBf) == dobj.Length):
                                if(acVidQ is not None):
                                    acVidQ.put(io.BytesIO(curBf)) #push on q
                            else: # trigger subsequent reads if bytes insufficient
                                contRead = True
                                curId = dobj.Id
                                remainingLen = dobj.Length - len(curBf)
                                canPopfromQ = True
                                #print('AcVid: subsequent triggered. remaining: ', remainingLen)
                                break

                        elif(dobj.Id == DataObjects.Ids.VideoData.value):
                            print('Got Video data')
                            curBf = data.read(dobj.Length)
                            if(len(curBf) == dobj.Length):
                                if(vidQ is not None):
                                    vidQ.put(io.BytesIO(curBf))
                            else: # trigger subsequent reads if bytes insufficient
                                contRead = True
                                curId = dobj.Id
                                remainingLen = dobj.Length - len(curBf)
                                canPopfromQ = True
                                #print('Vid: subsequent triggered. remaining: ', remainingLen)
                                break
                        
                        elif(dobj.Id == DataObjects.Ids.SpectrumData.value):
                            print('Got Spectrum data')
                            curBf = data.read(dobj.Length)
                            if(len(curBf) == dobj.Length):
                                if(specQ is not None):
                                    specQ.put(io.BytesIO(curBf)) #push on deque
                            else: # trigger subsequent reads if bytes insufficient
                                contRead = True
                                curId = dobj.Id
                                remainingLen = dobj.Length - len(curBf)
                                canPopfromQ = True
                                #print('Spec: subsequent triggered. remaining: ', remainingLen)
                                break

                        elif(dobj.Id == DataObjects.Ids.AudioData.value):
                            print('Got Audio data')
                            curBf = data.read(dobj.Length)
                            if(len(curBf) == dobj.Length):
                                if(audQ is not None):
                                    audQ.put(io.BytesIO(curBf)) #push on deque
                            else: # trigger subsequent reads if bytes insufficient
                                contRead = True
                                curId = dobj.Id
                                remainingLen = dobj.Length - len(curBf)
                                canPopfromQ = True
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
                                canPopfromQ = True
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
                                canPopfromQ = True
                                #print('Thermal: subsequent triggered. remaining: ', remainingLen)
                                break
                        
                        elif(dobj.Id == DataObjects.Ids.CommonStatus.value):
                            print('Got CommonStatus data')
                            curBf = data.read(dobj.Length)
                            if(len(curBf) == dobj.Length):
                                pass #TODO: update camera status
                            else: # trigger subsequent reads if bytes insufficient
                                contRead = True
                                curId = dobj.Id
                                remainingLen = dobj.Length - len(curBf)
                                canPopfromQ = True
                                #print('CommonStatus: subsequent triggered. remaining: ', remainingLen)
                                break

                        #check if inMsg still has stuff that can be read
                        curIdx = data.tell()
                        if(data.read(-1)):
                            data.seek(curIdx) #restore
                            canPopfromQ = False
                        else: #proceed normally
                            canPopfromQ = True 

    '''Decodes and Publishes Video data'''
    def publishVideo(self, q:Queue, canRun:bool, dataConn:Connection):
        print('Starting video decoder and publisher ...')

        saveVideo = False
        new_frame_t = 0
        prev_frame_t = 0

        if(saveVideo):
            vidOut = cv2.VideoWriter('soundcam.mp4', cv2.VideoWriter_fourcc(*'AVC1'), 5, (240, 320))
            #vidOut = cv2.VideoWriter('soundcam.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, (240, 320))
        def visualizeVideo(frame):
            nonlocal new_frame_t, prev_frame_t
            if(not saveVideo):
                color_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                font = cv2.FONT_HERSHEY_SIMPLEX
                new_frame_t = time.time()
                fps = 1/(new_frame_t - prev_frame_t)
                prev_frame_t = new_frame_t
                fps =  int(fps)
                cv2.putText(color_img, str(fps), (7, 70), font, 2, (100, 255, 0), 3, cv2.LINE_AA)
                #dataConn.send(color_img)
                cv2.imshow('Video Data', color_img)
                if cv2.waitKey(1) == ord('q'):
                    exit(-99)
            else:
                vidOut.write(frame)
        
        """ output = StreamingOutput()
        try:
            address = ('', 8000)
            print('HTTP stream ended!')
            server = StreamingServer(address, StreamingHandler)
            server.serve_forever()
        finally:
            print('HTTP stream ended!') """

        start_t = time.time()
        hits = 0
        dstr = '<Q2H'
        while(canRun):
            if(not q.empty()):
                #print('Video Q size: ', q.qsize())
                vidMsg:io.BytesIO = q.get()
                """ print('\nVideo msg: \n', vidMsg.read(8))
                break """
                vid = DataObjects.MDVideoData._make(struct.unpack(dstr, vidMsg.read(struct.calcsize(dstr))))
                #print(canRun, ' <- CanRun?\n--------------------------VIDEO DATA HDR ', vid, '\n')
                datasize = vid.VResolution * vid.HResolution
                vidframe:np.array = np.array(
                    struct.unpack('<' + str(datasize) +'B', vidMsg.read(-1)), dtype=np.uint8).reshape(vid.VResolution, vid.HResolution)
                
                if(self.pubObj.cfg['visualize_video']):
                    visualizeVideo(vidframe)
                #del vidMsg
                hits += 1
                if(time.time() - start_t >= 1.0):
                    print('==================================================================Vid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
        
        if(saveVideo):
            vidOut.release()
        cv2.destroyAllWindows()

    '''Decodes and Publishes Acoustic Video data'''
    def publishAcousticVideo(self, q:Queue, canRun:bool, dataConn:Connection):
        print('Starting acoustic video decoder and publisher ...')
        new_frame_t = 0
        prev_frame_t = 0
        
        def visualizeVideo(frame):
            nonlocal new_frame_t, prev_frame_t
            frame = np.interp(frame, 
                              (self.pubObj.cfg['minimum_db'], self.pubObj.cfg['maximum_db']), 
                              (0, 255))
            frame = frame.astype('uint8')
            mimg = np.zeros((48, 64, 3), dtype=np.uint8)
            cv2.applyColorMap(frame, cv2.COLORMAP_JET, mimg)
            #bigimg = mimg.repeat(10, axis=0).repeat(10, axis=1)
            rimg_up = cv2.resize(mimg, 
                                 (self.pubObj.cfg['camera_W'], self.pubObj.cfg['camera_H']), 
                                 interpolation= cv2.INTER_AREA)
            font = cv2.FONT_HERSHEY_SIMPLEX
            new_frame_t = time.time()
            fps = 1/(new_frame_t - prev_frame_t)
            prev_frame_t = new_frame_t
            fps =  int(fps)
            cv2.putText(rimg_up, str(fps), (7, 70), font, 2, (100, 255, 0), 3, cv2.LINE_AA)
            #dataConn.send(rimg_up) #send image out
            cv2.namedWindow("Acoustic Video", cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Acoustic Video', rimg_up)
            if cv2.waitKey(1) == ord('q'):
                exit(-99)

        start_t = time.time()
        hits = 0
        while(canRun):
            if(not q.empty()):
                acvidMsg:io.BytesIO = q.get()
                """ print('\nAcoustic-Video msg: \n', acvidMsg.read(8))
                break """
                dstr = '<Q6L'
                dssize = struct.calcsize(dstr)
                acvid = DataObjects.MDAcousticImageData._make(struct.unpack(dstr, acvidMsg.read(dssize)))
                acvidMsg.seek(dssize)
                #print('\n', acvid, '\n')
                frameData:np.array = np.array(
                    struct.unpack('<3072f', acvidMsg.read(-1)), dtype=np.float32).reshape(48, 64)
                #print(np.max(data), ' ', np.min(data))
                if(self.pubObj.cfg['visualize_acoustic_video']):
                    visualizeVideo((frameData))
                #del acvidMsg 
                hits += 1
                if(time.time() - start_t >= 1.0):
                    print('===============================================================AcousticVid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
        if(self.pubObj.cfg['visualize_acoustic_video']):
            cv2.destroyAllWindows()

    '''Decodes and Publishes Spectrum data'''
    def publishSpectrum(self, q:Queue, canRun:bool, dataConn:Connection):
        print('Starting spectrum decoder and publisher ...')
        while(canRun):
            if(not q.empty()):
                specMsg:io.BytesIO = q.get()
                """ print('\nSpectrum msg: \n', specMsg.read(8))
                break """
                dstr = '<QfH2L'
                dssize = struct.calcsize(dstr)
                spc = DataObjects.MDSpectrumData1._make(struct.unpack(dstr, specMsg.read(dssize)))
                #print('\n', spc, '\n')
                data_global:np.array = np.array(
                    struct.unpack('<1024f', specMsg.read(4096)), dtype=np.float32)
                data_local:np.array = np.array(
                    struct.unpack('<1024f', specMsg.read(4096)), dtype=np.float32)
                if(self.pubObj.cfg['visualize_spectrum']):
                    dataConn.send((data_global, data_local))
                del specMsg
        if(self.pubObj.cfg['visualize_spectrum']):
            try:
                dataConn.close()
            except:
                pass

    '''Decodes and Publishes Audio data'''
    def publishAudio(self, q:Queue, canRun:bool, dataConn:Connection):
        print('Starting audio decoder and publisher ...')
        start_t = time.time()
        hits = 0
        while(canRun):
            if(not q.empty()):
                audMsg:io.BytesIO = q.get()
                """ print('\nAudio msg: \n', audMsg.read(8))
                break """
                dstr = '<Q2h'
                aud1 = DataObjects.MDAudioData1._make(struct.unpack(dstr, audMsg.read(struct.calcsize(dstr))))
                data_plus = np.array(struct.unpack('<2048i', audMsg.read(8192)), dtype=np.int32)
                data_minus = np.array(struct.unpack('<2048i', audMsg.read(8192)), dtype=np.int32)
                audioData = data_plus + data_minus
                if(self.pubObj.cfg['visualize_audio']):
                    dataConn.send(audioData)
                
                dstr = '<dH2L'
                aud2 = DataObjects.MDAudioData2._make(struct.unpack(dstr, audMsg.read(-1)))
                #print('\n', aud1, aud2, '\n')
                #print('max audio: ', np.max(data_plus), ' Min: ', np.min(data_minus))
                #print(data_g)
                del audMsg
                hits += 1
                if(time.time() - start_t >= 1.0):
                    print('================================================================Audio @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
        if(self.pubObj.cfg['visualize_audio']):
            try:
                dataConn.close()
            except:
                pass

    '''Decodes and Publishes Raw data'''
    def publishRaw(self, q:Queue, canRun:bool):
        print('Starting raw data publisher (for debugging) ...')
        while(canRun):
            pass

    '''Decodes and Publishes Thermal Video data'''
    def publishThermalVideo(self, q:Queue, canRun:bool):
        print('Starting thermal video decoder and publisher ...')
        dstr = '<Q2H'
        while(canRun):
            if(not q.empty()):
                vidMsg:io.BytesIO = q.get()
                vid = DataObjects.MDThermalVideoData._make(struct.unpack(dstr, vidMsg.read(struct.calcsize(dstr))))
                #print(canRun, ' <- CanRun?\n--------------------------VIDEO DATA HDR ', vid, '\n')
                datasize = vid.VResolution * vid.HResolution
                print('Thermal dtype: ', vid.DataType, ' Datasize: ', datasize)
                


    '''Updates the plot'''
    def plotUpdater(self, frame):
        # update plot
        if(self.pubObj.cfg['visualize_audio']):
            self.plotLines[0].set_ydata(self.aud_recv.recv())
        if(self.pubObj.cfg['visualize_spectrum']):
            data = self.spec_recv.recv()
            self.plotLines[1].set_ydata(data[0])
            self.plotLines[2].set_ydata(data[1])
        return self.plotLines
    
    '''Displays live AUDIO/SPECTRUM data'''
    def displayLiveData(self):
        print('Starting visual display')
        fig, ax = plt.subplots(nrows=2, ncols=1, figsize=(8, 4))
        self.plotLines = list()
        
        ax[0].set_title('Audio Data')
        self.focusedAudio = np.zeros(2048, dtype=np.int32)
        x1 = np.linspace(1, 2048, 2048)
        ax[0].set_ylim(top=self.pubObj.cfg['maximum_value'], 
                       bottom=self.pubObj.cfg['minimum_value'])  # set safe limit to ensure that all data is visible.
        ax[0].set_facecolor((0.18, 0.176, 0.18))
        audlines, = ax[0].plot(x1, self.focusedAudio, color=(0.85, 1, 0.0))
        self.plotLines.append(audlines)

        ax[1].set_title('Spectrum Data')
        x2 = np.linspace(1, 1024, 1024)
        ax[1].set_ylim(top=self.pubObj.cfg['maximum_frequency'], 
                       bottom=self.pubObj.cfg['minimum_frequency'])  # set safe limit to ensure that all data is visible.
        ax[1].set_facecolor((1, 1, 1))
        sline1, = ax[1].plot(x2, np.zeros(1024, dtype=np.float32), color=(0.85, 1, 0.0))
        self.plotLines.append(sline1)
        sline2, = ax[1].plot(x2, np.zeros(1024, dtype=np.float32), color=(0.95, 0.4, 0.1))
        self.plotLines.append(sline2)
        
        self.visualUp = True #for logic in startStream
        ani = FuncAnimation(fig, self.plotUpdater, interval=10, blit=True, cache_frame_data=False)
        plt.show()

    def displayProcess(self):
        print('Starting display process')
        self.displayLiveData() #display audio/spectrum signal(s)

    ''' Post Processes data for comprehensive visualization '''
    def postProc(self):
        print('Starting post-processing process')
        while(True):
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
        

if __name__ == '__main__':
    camObj = SoundCamConnector(debug=True)
    atexit.register(camObj.disconnect)

    if(camObj.connect()):
        print('Sending Initial configuration and preparing state')
        camObj.configure()

    #camObj.restartCamera()
    time.sleep(8.0)
    camObj._startMeasurement()
    while(camObj.processData):
        time.sleep(0.001)
        #print('Stream running ...')
    
    #asyncio.run(camObj.asyncStartUp())