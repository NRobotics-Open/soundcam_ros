#!/usr/bin/env python3

'''
This package is developed by N Robotics GmbH to interface with the Sound Camera 
from CAE Sofware & Systems GmbH.

Developer: ephson@nrobotics.com
'''

import socket, time, atexit, os, io, sys, signal, subprocess, select, struct
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
    CameraProtocol, CameraProtocolProxy, CameraProtocolManager
import multiprocessing
from multiprocessing import Process, Semaphore
from queue import Empty, Queue
import SharedArray as sa
import psutil

from utils import SoundUtils as SU, SoundUtilsProxy as SUP, SoundUtilsManager, \
    EuclideanDistTracker, SignalInfo


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
        self.send_to_gui = False
        self.invokeId = 1
        self.cfgObj = cfgObj

        #debug parameters
        self.new_frame_t = 0
        self.prev_frame_t = 0

        #prepare utils manager
        # manager = SoundUtilsManager()
        # manager.start()
        # self.scamUtils = manager.SoundUtils()
        self.scamUtils = SU(window=60, detectionWin=5, acFreq=10, specFreq=4, 
                            dynamic_thresh_f=self.cfgObj['dynamic_thresh_factor'],
                            low_thresh_f=self.cfgObj['low_thresh_factor'],
                            smoothing_win=self.cfgObj['smoothing_win'], 
                            trigger_duration=self.cfgObj['trigger_duration'])
        self.objTracker = EuclideanDistTracker()
        
        self.shmnames=['npaudiomem', 'npglobalspecmem', 'nplocalspecmem', 'npacmem', 'npenergymem']

        self.scamUtils.setScalingMode(SU.ScalingMode(self.cfgObj['scalingMode']), 
                                      dynamic=self.cfgObj['dynamic'], 
                                      max=self.cfgObj['maximum'],
                                      crest= self.cfgObj['crest'],
                                      minF=self.cfgObj['minFrequency'],
                                      maxF=self.cfgObj['maxFrequency'])
        for shm in self.shmnames:
            try:
                sa.delete(shm) #audio buffer
            except:
                pass
        self.audioBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.AUDIO.value], (self.scamUtils.p_getAudLen(), 1))
        self.globalSpecBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.GLOBAL_SPECTRUM.value], (self.scamUtils.p_getWindow(), 1023))
        self.localSpecBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.LOCAL_SPECTRUM.value], (self.scamUtils.p_getWindow(), 1023))
        self.energyBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.ENERGY.value], (self.scamUtils.p_getSpecLen(), 1))
        self.acousticBuffer = sa.create("shm://" + self.shmnames[SU.BufferTypes.ACOUSTIC.value], (self.scamUtils.p_getACLen(), 1))
        self.scamUtils.initializeSharedBuffers(shmnamesls=self.shmnames)

        self.bufSize = 1460
        self.recvStream = False
        self.hasStreamData = False
        self.canRun = True
        self.processData = multiprocessing.Event()
        self.processData.set()
        self.visualUp = False
        self.connected = False
        self.is_alive = False
        self.signalInfo:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)

        #prepare queues
        self.processes = list()
        self.threads = list()
        self.queues = list()
        self.qDict = dict()

        #media Q setup
        self.vidQ = None
        self.acVidQ = None
        self.specQ = None
        self.audQ = None
        self.thermalQ = None

        #post-processing setup
        self.overlayBA = bitarray(self.cfgObj['overlayConfig'])
        self.vidBW = None
        self.vidAC = None
        self.vidTm = None
        self.spectrum = None
        self.audio = None

        self.proc_vidOBW = Queue(maxsize=3)
        self.proc_acVidQ = Queue(maxsize=3)
        self.proc_specQ = Queue(maxsize=3)
        self.proc_audQ = Queue(maxsize=3)
        self.proc_thmQ = Queue(maxsize=3)
            
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
            self.vidQ = Queue(maxsize=100)
            self.queues.append(self.vidQ)
            self.threads.append(Thread(target=self.processVideo, daemon=True))
            #self.processes.append(Process(target=self.processVideo, args=(self.vidQ, self.processData), name='vidProc'))
        if(self.cfgObj['processAcVideo']):
            self.acVidQ = Queue(maxsize=100)
            self.queues.append(self.acVidQ)
            self.ac_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processAcVideo, daemon=True))
            # self.processes.append(Process(target=self.processAcVideo, 
            #                               args=(self.acVidQ, self.processData, 
            #                                     self.scamUtils, self.ac_semaphore)))
        if(self.cfgObj['processSpectrum']):
            self.specQ = Queue(maxsize=100)
            self.queues.append(self.specQ)
            self.spec_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processSpectrum, daemon=True))
            # self.processes.append(Process(target=self.processSpectrum, 
            #                               args=(self.specQ, self.processData, 
            #                                     self.scamUtils, self.spec_semaphore)))
        if(self.cfgObj['processAudio']):
            self.audQ = Queue(maxsize=100)
            self.queues.append(self.audQ)
            self.aud_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processAudio, daemon=True))
            # self.processes.append(Process(target=self.processAudio, 
            #                               args=(self.audQ, self.processData, 
            #                                     self.scamUtils, self.aud_semaphore)))
        if(self.cfgObj['processThermal']):
            self.thermalQ = Queue(maxsize=100)
            self.queues.append(self.thermalQ)
            self.threads.append(Thread(target=self.processThermalVideo, daemon=True))
            # self.processes.append(Process(target=self.processThermalVideo, 
            #                               args=(self.thermalQ, self.processData)))
        # if(self.cfgObj.p_pubRaw):
        #     self.rawQ = Queue()
        #     self.qDict[SoundCamConnector.DTypes.RAW.name] = self.rawQ
        #     self.processes.append(Process(target=self.publishRaw, args=(self.rawQ, self.processData)))
        
        if(self.cfgObj['visualizeAudio'] or self.cfgObj['visualizeSpectrum']):
            self.processes.append(Process(target=self.displayProcess, args=(self.processData,)))

        #prepare protocol manager
        # self.protocmanager = CameraProtocolManager()
        # self.protocmanager.start()
        # self.protocol = self.protocmanager.CameraProtocol(protocol=self.cfgObj['protocol'], debug=self.debug)
        self.protocol = CameraProtocol(protocol=self.cfgObj['protocol'], debug=self.debug)

        #start threads
        # self.globalQ = Queue(maxsize=65)
        # self.queues.append(self.globalQ)
        self.threads.append(Thread(target=self.receiveCyclic3, daemon=True))
        #self.threads.append(Thread(target=self.streamFilter))
        # Start the memory monitor in the main process
        # if(self.debug):
        #     self.threads.append(Thread(target=self.monitor_memory, daemon=True))
        for th in self.threads:
            th.start()
        
        #start processes
        print('Starting processes ...')
        # self.processes.append(Process(target=self.streamFilter, args=(self.qDict, self.processData)))
        for proc in self.processes:
            proc.start()
        

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
            self.is_alive = True
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
            #self.sock.setblocking(False)
            self.connected = True
        except Exception as ex:
            print('Failure Establishing connection to device! ', ex)
            self.connected = False
            return False
        print("\nSocket initialized on: \t IP| %s PORT| %d" % (ip_addr, port))
        print("Clearing buffer ...")
        self.emptyBuffer()
        print('Fetching device Id ...')
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
        #disable cyclic loop
        self.recvStream = False
        self.connected = False
        self.is_alive = False
        #send reset command
        query = self.protocol.generateReadRequest(command=CommandCodes.ResetReq, 
                                                  invokeId=self.invokeId)
        self.sendData(query=query)
        #disable initial status flag
        self.protocol.setInitialStatus(False)
        #clear buffers
        self.scamUtils.resetBuffers()
        for _,q in self.qDict.items():
            self._clearQueue(q)

        return True
    
    def _clearQueue(self, q:Queue):
        while(not q.empty()):
            try:
                q.get_nowait()
            except Exception:
                pass
    
    def _addQueue(self, q:Queue, data:bytes):
        if(q is not None):
            try:
                q.put(data, block=False)
            except:
                pass

    def disconnect(self):
        if(not self.processData.is_set()):
            return
        
        self.canRun = False
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
            for _,q in self.qDict.items():
                q.put(None)
            
            print('Joining threads ...')
            for th in self.threads:
                if(th.is_alive()):
                    th.join(timeout=1)
            print('Joining processes ...')
            # self.protocmanager.join(timeout=1.0)
            # print('Manager ended!')
            for p in self.processes:
                if(p.is_alive()):
                    #print(f'Terminating {p.name}')
                    p.terminate()
                    p.join(timeout=1)  # Wait up to 1 second for process to terminate
                    if(p.is_alive()):
                        #print(f'Force killing {p.name}')
                        os.kill(p.pid, signal.SIGKILL)
                        p.join()
        except Exception as ex:
            print('Error joining processes/threads: ', ex)
    
    ''' Release shared memory blocks '''
    def release_shared(self):
        for i, shm in enumerate(self.shmnames):
            #self.semaphores[i].acquire()
            try:
                sa.delete(shm)
            except FileNotFoundError:
                pass
            except Exception:
                pass
            #self.semaphores[i].release()
        #del self.semaphores

    def signal_handler(self, sig, frame):
        print("Signal received, shutting down...")
        self.disconnect()
        self.release_shared()
        exit(0)

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
            self.invokeId += 1

            #concatenate PrepareState
            query += self.protocol.setState(self.invokeId, 
                                            Device.DeviceStates(self.cfgObj['deviceState']))#set device state
            self.invokeId += 1

            #concatenate DataToSend
            query += self.protocol.dataToSendConfig(self.invokeId, 
                                                    dataToSend1=self.cfgObj['dataToSend1'],
                                                    dataToSend2=self.cfgObj['dataToSend2'])
            self.invokeId += 1
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
        self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                          max=maximum, crest=crest, minF=minFreq, maxF=maxFreq)
        self.scamUtils.resetBuffers()
        return True
        self.hasStreamData = False
        try:
            #configure acoustic filter
            self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                          max=maximum, crest=crest, minF=minFreq, maxF=maxFreq)
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
            #concatenate DataToSend
            self.invokeId += 1
            query += self.protocol.dataToSendConfig(self.invokeId, 
                                                    dataToSend1=self.cfgObj['dataToSend1'],
                                                    dataToSend2=self.cfgObj['dataToSend2'])
            self.invokeId += 1
            self.sendData(query=query)
            self.scamUtils.resetBuffers()
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
            # query = camObj.protocol.writeVidFPS(self.invokeId, self.cfgObj['videoFPS'])
            # self.invokeId += 1
            query = self.protocol.startStopProcedure(self.invokeId) #start procedure
            self.invokeId += 1
            self.sendData(query=query)
            self.recvStream = True
            print('Starting measurement ...')
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
            print('Stopped!')
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
        while(self.processData.is_set() and self.canRun):
            if(self.connected):
                try:
                    res = self.sock.recv(self.bufSize)
                    if(res):
                        if(self.recvStream):
                            #self._addQueue(self.globalQ, res, cast=False)
                            self.globalQ.put(res)
                            if(not self.hasStreamData):
                                self.hasStreamData = True
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
                except Exception:
                    pass
            else:
                #print('Awaiting socket connnection ...')
                time.sleep(0.1)
    
    def emptyBuffer(self):
        """remove the data present on the socket"""
        input = [self.sock]
        while 1:
            inputready, o, e = select.select(input,[],[], 0.0)
            if len(inputready)==0: break
            for s in inputready: s.recv(1)

    def recv_all(self, num_bytes):
        """Receive exactly num_bytes from the socket."""
        data = b''
        while len(data) < num_bytes:
            packet = self.sock.recv(num_bytes - len(data))  # Receive the remaining amount of data
            if not packet:
                # The connection was closed prematurely
                raise ConnectionError("Socket connection closed before receiving all data")
            data += packet
        return data

    def receiveCyclic2(self):
        print('Cyclic[2] thread started ...')
        start_t = time.time()
        dstr_cmd_invkid = '<2B'
        dstr_hdr = '<H2L'
        gib_cnt = 0
        raw_buffer = b''
        while(self.processData.is_set() and self.canRun):
            '''
                Read the first 2 bytes (Command & Invoke Id)
                    if Req/Data message:
                        Read the next 4 bytes (Data Length)
                    if Response message:
                        Read the next 8 bytes (Error Code & Data Length)
                
                Read the corresponding data block & wrap in memoryview
                Pass data id and block to function for immediate decoding
                Repeat
            '''
            if(self.connected):
                try:
                    readable, _, _ = select.select([self.sock], [], [], 0.5)  # 2.0 is the timeout in seconds
                    if(self.sock in readable):
                        if(self.recvStream):
                            self.hasStreamData = True
                            res = self.sock.recv(2)
                            cmd_obj = struct.unpack(dstr_cmd_invkid, res) #decode command & invoke id
                            if(cmd_obj[0] == DataMessages.CommandCodes.DataMessage.value):
                                #print('Stub 1')
                                res_ext = self.sock.recv(10)
                                _, datalen, objcnt = struct.unpack(dstr_hdr, res_ext)
                                res += res_ext
                                raw_buffer += res
                                print('DataMessage | InvokeId', cmd_obj, ' | Len: ',datalen, ' Cnt: ', objcnt, '\n', res.hex())
                                if(objcnt > 1):
                                    print('DataMessage contains multiple objects')
                                    pass
                                #   exit(-9)
                                hdr = self.sock.recv(8)
                                raw_buffer += hdr
                                objhdr = self.protocol.unpackDataObjectHeader(hdr)
                                print('Got object with Header ->',hdr.hex(),  '| (Type: %i, Version: %i, Length: %i) ' % objhdr)
                                #Reading Data object
                                datablock = self.recv_all(objhdr[2])
                                dblocklen = len(datablock)
                                if(dblocklen != objhdr[2]):
                                    print('Data read length mismatch: missing bytes = ', (objhdr[2] - dblocklen))
                                raw_buffer += datablock
                                if(objhdr[0] == DataObjects.Ids.VideoData.value): #video
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n\n')
                                    pass
                                elif(objhdr[0] == DataObjects.Ids.AcousticVideoData.value): #acoustic video
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n')
                                    pass
                                elif(objhdr[0] == DataObjects.Ids.SpectrumData.value): #spectrum
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n')
                                    pass
                                elif(objhdr[0] == DataObjects.Ids.AudioData.value): #audio
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n')
                                    pass
                                elif(objhdr[0] == DataObjects.Ids.ThermalVideoData.value): #thermal video
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n')
                                    pass
                                elif(objhdr[0] == DataObjects.Ids.CommonStatus.value): #common status
                                    print(datablock[-1024:].hex())
                                    print('block read success!\n')
                                    pass
                                else: #raw probably data
                                    # print(res, ' | ',cmd_obj)
                                    # print('Got Raw block ---')
                                    #print(datablock.hex())
                                    print('block read failure!\n')
                                    #print(raw_buffer.hex())
                                    raw_buffer = b''
                                    gib_cnt += 1
                                    if(gib_cnt >= 10):
                                        break

                            elif((cmd_obj[0] == CommandCodes.PrepareStateRes.value) or 
                                 (cmd_obj[0] == CommandCodes.FinishStateRes.value) or 
                                 (cmd_obj[0] == CommandCodes.StartProcedureRes.value) or 
                                 (cmd_obj[0] == CommandCodes.WriteDataObjectRes.value) or 
                                 (cmd_obj[0] == CommandCodes.StopProcedureRes.value) or 
                                 (cmd_obj[0] == CommandCodes.ResetRes.value)):
                                res = self.sock.recv(10)
                                hdr_obj = struct.unpack(dstr_hdr, res)
                                #print(res, ' | ',hdr_obj)
                            
                        else: #if not streaming
                            try: #TODO: revisit this in the future
                                res = self.sock.recv(self.bufSize)
                                self.protocol.unpackDecodeResponse(response=res)
                            except Exception as ex:
                                print('\nError Unpacking and Decoding ...')

                    now = time.time()
                    
                    if((now - start_t) >= 10.0 and self.connected and self.debug):
                        print('Cyclic loop running ...')
                        start_t = time.time()
                except Exception:
                    pass
            else:
                #print('Awaiting socket connnection ...')
                time.sleep(0.1)
        exit(-9)


    def receiveCyclic3(self):
        print('Cyclic[3] thread started ...')
        start_t = time.time()
        dstr_cmd_invkid = '<2B'
        dstr_hdr = '<H2L'
        gib_cnt = 0
        #raw_buffer = b''
        while(self.processData.is_set() and self.canRun):
            '''
                Read the first 2 bytes (Command & Invoke Id)
                    if Req/Data message:
                        Read the next 4 bytes (Data Length)
                    if Response message:
                        Read the next 8 bytes (Error Code & Data Length)
                
                Read the corresponding data block & wrap in memoryview
                Pass data id and block to function for immediate decoding
                Repeat
            '''
            if(self.connected):
                try:
                    if(self.recvStream):
                        self.hasStreamData = True
                        res = self.sock.recv(2)
                        cmd_obj = struct.unpack(dstr_cmd_invkid, res) #decode command & invoke id
                        if(cmd_obj[0] == DataMessages.CommandCodes.DataMessage.value):
                            #print('Stub 1')
                            res_ext = self.sock.recv(10)
                            _, datalen, objcnt = struct.unpack(dstr_hdr, res_ext)
                            res += res_ext
                            #raw_buffer += res
                            #print('DataMessage | InvokeId', cmd_obj, ' | Len: ',datalen, ' Cnt: ', objcnt, '\n', res.hex())
                            if(objcnt > 1):
                                print('DataMessage contains multiple objects')
                                exit(-9)
                            hdr = self.sock.recv(8)
                            #raw_buffer += hdr
                            objhdr = self.protocol.unpackDataObjectHeader(hdr)
                            #print('Got object with Header ->',hdr.hex(),  '| (Type: %i, Version: %i, Length: %i) ' % objhdr)
                            #Reading Data object
                            datablock = self.recv_all(objhdr[2])
                            dblocklen = len(datablock)
                            if(dblocklen != objhdr[2]):
                                print('Data read length mismatch: missing bytes = ', (objhdr[2] - dblocklen))
                                exit(-9)
                            
                            #raw_buffer += datablock
                            if(objhdr[0] == DataObjects.Ids.VideoData.value): #video
                                self._addQueue(self.vidQ, datablock)
                            elif(objhdr[0] == DataObjects.Ids.AcousticVideoData.value): #acoustic video
                                self._addQueue(self.acVidQ, datablock)
                            elif(objhdr[0] == DataObjects.Ids.SpectrumData.value): #spectrum
                                self._addQueue(self.specQ, datablock)
                            elif(objhdr[0] == DataObjects.Ids.AudioData.value): #audio
                                self._addQueue(self.audQ, datablock)
                            elif(objhdr[0] == DataObjects.Ids.ThermalVideoData.value): #thermal video
                                self._addQueue(self.thermalQ, datablock)
                            elif(objhdr[0] == DataObjects.Ids.CommonStatus.value): #common status
                                #TODO: call function to decode
                                pass
                            else: #raw probably data
                                # print(res, ' | ',cmd_obj)
                                # print('Got Raw block ---')
                                #print(datablock.hex())
                                print('block read failure!\n')
                                exit(-9)
                                #print(raw_buffer.hex())
                                # raw_buffer = b''
                                # gib_cnt += 1
                                # if(gib_cnt >= 10):
                                #     break

                        elif((cmd_obj[0] == CommandCodes.PrepareStateRes.value) or 
                                (cmd_obj[0] == CommandCodes.FinishStateRes.value) or 
                                (cmd_obj[0] == CommandCodes.StartProcedureRes.value) or 
                                (cmd_obj[0] == CommandCodes.WriteDataObjectRes.value) or 
                                (cmd_obj[0] == CommandCodes.StopProcedureRes.value) or 
                                (cmd_obj[0] == CommandCodes.ResetRes.value)):
                            res = self.sock.recv(10)
                            hdr_obj = struct.unpack(dstr_hdr, res)
                            #print(res, ' | ',hdr_obj)
                        
                    else: #if not streaming
                        try: #TODO: revisit this in the future
                            res = self.sock.recv(self.bufSize)
                            self.protocol.unpackDecodeResponse(response=res)
                        except Exception as ex:
                            print('\nError Unpacking and Decoding ...')

                    now = time.time()
                    
                    if((now - start_t) >= 10.0 and self.connected and self.debug):
                        print('Cyclic loop running ...')
                        start_t = time.time()
                except Exception:
                    pass
            else:
                #print('Awaiting socket connnection ...')
                time.sleep(0.1)
        exit(-9)
    

    def _routeData(self, curId, curBf, rawQ=None):
        if curId == DataObjects.Ids.AcousticVideoData.value:
            self._addQueue(self.acVidQ, curBf)
        elif curId == DataObjects.Ids.VideoData.value:
            self._addQueue(self.vidQ, curBf)
        elif curId == DataObjects.Ids.SpectrumData.value:
            self._addQueue(self.specQ, curBf)
        elif curId == DataObjects.Ids.AudioData.value:
            self._addQueue(self.audQ, curBf)
        elif curId == DataObjects.Ids.ThermalVideoData.value:
            self._addQueue(self.thermalQ, curBf)
        elif curId == DataObjects.Ids.RawData.value and rawQ is not None:
            self._addQueue(rawQ, curBf)
        elif curId == DataObjects.Ids.CommonStatus.value:
            pass
    
    def _readBytes(self, inBuff:memoryview, curReadPos:int, bufLen:int):
        rdLen = len(inBuff) - curReadPos
        try:
            if(rdLen < bufLen):
                return (inBuff[curReadPos: (curReadPos + rdLen)].tobytes(), (curReadPos + rdLen), rdLen)
            return (inBuff[curReadPos:(curReadPos + bufLen)].tobytes(), (curReadPos + bufLen), bufLen)
        except Exception as e:
            print('Error reading bytes from memoryview object')
            exit(99)
        
    '''Filters the generic Deque to the corresponding Deques based on the type'''
    def streamFilter(self):
        print('Starting stream filter ...')
        canPopfromQ:bool = True
        contRead:bool = False
        curBf = bytes()
        xsData = bytes()
        inObj = bytes()
        curId:int = 0
        remainingLen:int = 0
        readPos:int = 0
        
        #get queues
        rawQ = None

        debug = False

                  
        dmsg:MDDataMessage = None
        isHdrPkt = False
        canPrintLen = True

        while(self.canRun):
            #read queue and filter
            if(not self.globalQ.empty()):
                if(not self.protocol.p_hasInitialStatus()):
                    if(debug):
                        print('Clearing Q: camera not initialized!')
                    self._clearQueue(self.globalQ)
                # if(globalQ.qsize() > 50):
                #     print('Global Queue Size: ', globalQ.qsize())
                
                if(canPopfromQ):
                    if(debug):
                        print('Popping from Queue ...')
                    try:
                        inObj = self.globalQ.get_nowait()
                    except Empty:
                        continue
                    sz = len(inObj)
                    if(canPrintLen and (sz == self.bufSize)):
                        isHdrPkt = True
                    
                    if((sz < self.bufSize) and not isHdrPkt):
                        canPrintLen = True
                    
                    if(xsData):
                        inObj = xsData + inObj
                        xsData = bytes()
                        sz = len(inObj)
                        if(debug):
                            print('Prepended xsData ...')
                    else:
                        data = memoryview(inObj)
                        readPos = 0
                
                    if(contRead):#if fetching subsequent bytes #TODO revisit this logic with readPos parameter
                        if(debug):
                            print('Fetching subsequent for ObjectType: ', curId)
                        rd, readPos, readsz = self._readBytes(data, readPos, remainingLen)
                        #print('SubReadSize: ', readsz, '\nSubsequent:  ', rd)
                        curBf += rd
                        if(readsz == remainingLen):
                            if(debug):
                                print('Appending to corresponding deque ...')
                            self._routeData(curId=curId, curBf=curBf)
                            #reset params
                            curId = 0
                            curBf = bytes()
                            contRead = False 
                            remainingLen = 0
                            if(remainingLen == sz):
                                continue
                        else: #if not all bytes fetched, recalculate remaining length
                            remainingLen = remainingLen - readsz
                            if(debug):
                                print('More data required. RemainingLength: ', remainingLen)
                            canPopfromQ = True
                            continue #go back to start
                else:
                    #print('previous Queue processing ...')
                    #reset buffers
                    if(readPos != 0):
                        inObj = data[readPos:].tobytes()
                        data = memoryview(inObj)
                        readPos = 0
                    canPrintLen = True
                    isHdrPkt = True
                    #print('\nPrev Content: ', inObj)
                
                if(canPrintLen and isHdrPkt):
                    if(inObj):
                        hdr, readPos, _ = self._readBytes(data, readPos, 12)
                        if(hdr == b'' or len(hdr) < 12):
                            canPopfromQ = True
                            continue
                        dmsg = self.protocol.unpackDataMessage(hdr)
                        if(dmsg is None):
                            canPopfromQ = True
                            continue
                        if(debug):
                            print(dmsg)
                    else:
                        if(debug):
                            print('msg failure! -> empty buffer')
                        canPopfromQ = True
                        continue
                    if(debug):
                        print('Pre-dmsg (if)', dmsg)
                    if((dmsg.Command == DataMessages.CommandCodes.DataMessage.value) and 
                       (dmsg.ObjectCount <= 10)):
                        for i in range(dmsg.ObjectCount):
                            dt, readPos, _ = self._readBytes(data, readPos, 8)
                            msg = self.protocol.unpackDataObjectHeader(dt)
                            if(msg is None):
                                canPopfromQ = True
                                continue
                            try:
                                 dobj:DataObjects.DataObjHeader = DataObjects.DataObjHeader._make(msg)
                            except Exception:
                                print('object header returned None!')
                                time.sleep(3.0)
                                exit(-99)  
                                    
                            curBf, readPos, readsz = self._readBytes(data, readPos, dobj.Length)
                            if(dobj.Id == DataObjects.Ids.AcousticVideoData.value):
                                #print('Got Acoustic-Video data')
                                if(readsz == dobj.Length):
                                    self._addQueue(self.acVidQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('AcVid: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.VideoData.value):
                                #print('Got Video data')
                                if(readsz == dobj.Length):
                                    self._addQueue(self.vidQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('Vid: subsequent triggered. remaining: ', remainingLen)
                                    break
                            
                            elif(dobj.Id == DataObjects.Ids.SpectrumData.value):
                                #print('Got Spectrum data')
                                if(readsz == dobj.Length):
                                    self._addQueue(self.specQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('Spec: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.AudioData.value):
                                #print('Got Audio data')
                                if(readsz == dobj.Length):
                                    self._addQueue(self.audQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('Audio: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.RawData.value):
                                #print('Got Raw data')
                                if(readsz == dobj.Length):
                                    self._addQueue(rawQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('Raw: subsequent triggered. remaining: ', remainingLen)
                                    break

                            elif(dobj.Id == DataObjects.Ids.ThermalVideoData.value):
                                #print('Got Thermal-Video data')
                                if(readsz == dobj.Length):
                                    self._addQueue(self.thermalQ, curBf)
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('Thermal: subsequent triggered. remaining: ', remainingLen)
                                    break
                            
                            elif(dobj.Id == DataObjects.Ids.CommonStatus.value):
                                #print('Got CommonStatus data')
                                if(readsz == dobj.Length): #TODO: update camera status
                                    #print(data.read(dobj.Length))
                                    pass
                                else: # trigger subsequent reads if bytes insufficient
                                    contRead = True
                                    curId = dobj.Id
                                    remainingLen = dobj.Length - readsz
                                    if(debug):
                                        print('CommonStatus: subsequent triggered. remaining: ', remainingLen)
                                    break

                            #check if inMsg still has stuff that can be read
                            if((len(data) - readPos) > 0):
                                canPopfromQ = False
                            else: #proceed normally
                                canPopfromQ = True
                        
                        #print('For loop break')
                        canPrintLen = False
                        isHdrPkt = False
                        canPopfromQ = True
                    else:
                        #print('\nPacket discarded! - Size: ', len(inObj))
                        res = self.getMatch(inObj)
                        if(res):
                            idx = res.span()[0] - 12
                            #print(res, ' Hdr at: ', idx)
                            readPos = idx
                            canPopfromQ = False
                        else:
                            canPopfromQ = True
            else:
                pass

    '''Decodes and Publishes Video data'''
    def processVideo(self):
        print('Starting video processor ...')

        start_t = time.time()
        hits = 0
        acFrame = None
        while(self.processData.is_set()):
            try:
                #print('Video Q size: ', q.qsize())
                raw = self.vidQ.get(timeout=0.1)
                if(raw is None):
                    continue
                _, self.vidBW = self.protocol.unpackDecodeVideoData(raw)

                # if(self.cfgObj['visualizeVideo']):
                #     if(self.cfgObj['visualizeOverlay'] and not self.proc_acVidQ.empty()):
                #         acFrame = self.proc_acVidQ.get()
                #     self.visualizeBWVideo(decoded[1], acFrame=acFrame)
                # else:
                #     pass
                #     # Update BW Queue
                #     # if(not self.proc_vidBW.full()): #push data to Q
                #     #     self.proc_vidBW.put(decoded[1])
                #     # else:
                #     #     self._clearQueue(self.proc_vidBW)
                #     #     self.proc_vidBW.put(decoded[1])

                #     # Update Overlay Queue
                #     # if(not self.proc_acVidQ.empty()):
                #     #     print('Got acoustic')
                #     #     res = self.visualizeBWVideo(decoded[1], 
                #     #             acFrame=self.proc_acVidQ.get(timeout=1.0), 
                #     #             getResult=True)
                #     #     if(not self.proc_vidOBW.full()): #push data to Q
                #     #         self.proc_vidOBW.put(res)
                #     #     else:
                #     #         self._clearQueue(self.proc_vidOBW)
                #     #         self.proc_vidOBW.put(res)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('==================================================================Vid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Empty:
                continue
            except Exception as e:
                print(f"Error in video process {e}")
                break

    '''Decodes and Publishes Acoustic Video data'''
    def processAcVideo(self):
        print('Starting acoustic video processor ...')

        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                raw = self.acVidQ.get(timeout=0.1)
                if(raw is None):
                    continue
                norm0_1 = self.scamUtils.updateAcousticBuffer(self.protocol.unpackDecodeAcousticVideoData(raw))
                if(self.cfgObj['visualizeAcVideo']):
                    self.visualizeACVideo(norm0_1)
                self.vidAC = self.visualizeACVideo(norm0_1, getResult=True)
                # if(res is not None):
                #     if(not self.proc_acVidQ.full()): #push acoustic data to overlay_Q
                #         self.proc_acVidQ.put(res)
                #     else:
                #         self._clearQueue(self.proc_acVidQ)
                #         self.proc_acVidQ.put(res)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('===============================================================AcousticVid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Empty:
                continue
            except Exception as e:
                print(f"Error in acoustic process {e}")
                break

    '''Decodes and Publishes Spectrum data'''
    def processSpectrum(self):
        print('Starting spectrum processor ...')

        saveSpectrum = False
        specData = []
        t = 30 #seconds
        record_t = time.time()
        #spcQ: Queue = self.qDict[SoundCamConnector.DTypes.O_SPEC.name]
        hits = 0
        start_t = time.time()

        # GUI
        if(self.send_to_gui):
            gui_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            gui_socket.connect((self.cfgObj['gui_hostname'], self.cfgObj['gui_port']))
        while(self.processData.is_set()):
            try:
                self.spectrum = self.protocol.unpackDecodeSpectrumData(self.specQ.get(timeout=0.05))
                if(self.spectrum is None):
                    continue
                with self.spec_semaphore:
                    self.scamUtils.updateSpectrumBuffer(self.spectrum)
                    self.signalInfo = self.scamUtils.getSignalAnalysis()
                
                if(self.send_to_gui):
                    message = 'This is a test message to GUI'
                    gui_socket.sendall(message.encode('utf-8'))
                                       
                #print('Spectrum:', decoded[0], ' ' ,decoded[1])
                # if(not self.proc_specQ.full()): #push spectrum data to overlay_Q
                #     self.proc_specQ.put(decoded)
                # else:
                #     self._clearQueue(self.proc_specQ)
                #     self.proc_specQ.put(decoded)
                if(saveSpectrum):
                    specData.append(self.spectrum[2])
                    if(time.time() - record_t >= t):
                        sr = int(self.spectrum[0][1] * 1024 * 2) #nyquist
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
            except Empty:
                continue
            except Exception as e:
                print(f"Error in spectrum process {e}")
                break
        if(self.send_to_gui):
            self.gui_socket.close()

    '''Decodes and Publishes Audio data'''
    def processAudio(self):
        print('Starting audio processor ...')

        saveAudio = False
        audioData_plus = []
        audioData_minus = []
        t = 30 #seconds
        record_t = time.time()

        #audQ:Queue = self.qDict[SoundCamConnector.DTypes.O_AUD.name]
        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                raw = self.audQ.get(timeout=0.1)
                if(raw is None):
                    continue
                self.audio = self.protocol.unpackDecodeAudioData(raw)
                with self.aud_semaphore:
                    self.scamUtils.updateAudioBuffer(self.audio) #update ringbuffer
                # if(not self.proc_audQ.full()): #push audio data to overlay_Q
                #     self.proc_audQ.put(decoded)
                # else:
                #     self._clearQueue(self.proc_audQ)
                #     self.proc_audQ.put(decoded)
                if(saveAudio):
                    audioData_plus.append(self.audio[2])
                    audioData_minus.append(self.audio[3])
                    if(time.time() - record_t >= t):
                        sr = 1/ self.audio[1][0]
                        print('\n--------------Saving audio with sampling frequency (Hz): ', sr)
                        audioData_plus = np.concatenate(audioData_plus)
                        audioData_minus = np.concatenate(audioData_minus)
                        #audioData /= np.max(np.abs(audioData),axis=0)
                        #self.scamUtils.writeAudio('outputAudio_single_plus.wav', audioData_plus, sr)
                        #self.scamUtils.writeAudio('outputAudio_single_minus.wav', audioData_minus, sr)
                        self.scamUtils.writeAudio('outputLong.wav', (audioData_plus + audioData_minus), sr)
                        saveAudio = False
                        
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Audio @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Empty:
                continue
            except Exception as e:
                print(f"Error in process {e}")
                break


    '''Decodes and Publishes Raw data'''
    def publishRaw(self, q:Queue, canRun:multiprocessing.Event):
        print('Starting raw data publisher (for debugging) ...')
        while(canRun.is_set()):
            pass

    '''Decodes and Publishes Thermal Video data'''
    def processThermalVideo(self):
        print('Starting thermal video processor ...')
        #tmQ:Queue = self.qDict[SoundCamConnector.DTypes.O_THM.name]
        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                raw = self.thermalQ.get(timeout=0.1)
                if(raw is None):
                    continue
                decoded = self.protocol.unpackDecodeThermalVideoData(raw)
                if(self.cfgObj['visualizeThermal']):
                    self.visualizeThermal(decoded[1])
                else:
                    self.vidTm = self.visualizeThermal(decoded[1], getResult=True)
                    # if(not self.proc_thmQ.full()): #push data to Q
                    #     self.proc_thmQ.put(res)
                    # else:
                    #     self._clearQueue(self.proc_thmQ)
                    #     self.proc_thmQ.put(res)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Thermal @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Empty:
                continue
            except Exception as e:
                print(f"Error in process {e}")
                break
    
    def monitor_memory(self):
        process = psutil.Process(os.getpid())
        while(self.processData.is_set()):
            mem_info = process.memory_info()
            print(f"Memory usage: RSS={mem_info.rss / (1024 * 1024)} MB")
            time.sleep(5)

    '''Updates the plot'''
    def plotUpdater(self, frame):
        if(self.cfgObj['visualizeAudio']):
            try:
                data = self.proc_audQ.get(timeout=0.1)
                if(data is not None):
                    audio = data[2] + data[3]
                    lims = np.max(audio) + 20000 # plus buffer range
                    self.ax[0].set_ylim(top=lims, bottom=-lims)
                    self.plotLines[0].set_ydata(audio)
            except Empty:
                pass
        
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
            try:
                data = self.proc_specQ.get(timeout=0.1)
                if(data is not None):
                    self.plotLines[1].set_ydata(data[2]) #global
                    self.plotLines[2].set_ydata(data[3]) #local
                    # spectogram plotting
                    self.ax_img = self.scamUtils.updateSpectogramPlot()
            except Empty:
                pass
            return self.plotLines, self.ax_img
        return self.plotLines
    

    
    '''Displays live AUDIO/SPECTRUM data'''
    def displayLiveData(self, canRun:multiprocessing.Event):
        print('Starting visual display')
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
    
    ''' Return camera alive status '''
    def isAlive(self):
        return self.is_alive

    ''' Returns the BW Video frame '''
    def getBWVideo(self):
        return self.vidBW
    
    # def getBWVideo(self):
    #     try:
    #         return self.proc_vidBW.get(timeout=0.1)
    #     except Empty:
    #         return None
    #     except Exception:
    #         return None
    
    ''' Returns the Overlayed video frame '''
    def getOverlayVideo(self):
        try:
            return self.proc_vidOBW.get(timeout=0.1)
        except Empty:
            return None
        except Exception:
            return None
    
    ''' Returns the Acoustic Video frame '''
    def getACVideo(self):
        return self.vidAC
    
    # def getACVideo(self):
    #     try:
    #         return self.proc_acVidQ.get(timeout=0.1)
    #     except Empty:
    #         return None
    #     except Exception:
    #         return None
    
    ''' Returns the Thermal Video frame '''
    def getTMVideo(self):
        return self.vidTm
    
    # def getTMVideo(self):
    #     try:
    #         return self.proc_thmQ.get(timeout=0.1)
    #     except Empty:
    #         return None
    #     except Exception:
    #         return None
    
    ''' Returns the Audio frame '''
    def getAudio(self):
        with self.aud_semaphore:
            return self.scamUtils.getAudioBuffer()[-2048:]
        
    ''' Returns the Audio Info '''
    def getAudioInfo(self):
        try:
            if(self.audio is not None):
                meta:DataObjects.MDAudioData2 = DataObjects.MDAudioData2._make(self.audio[1])
                srate = int(1/meta.dt)
                return dict({'channels': 1,
                            'sample_rate': srate,
                            'bitrate': 16 * srate})
        except Empty:
            return None
        except Exception:
            return None

    # def getAudioInfo(self):
    #     try:
    #         decoded = self.proc_audQ.get(timeout=0.1)
    #         if(decoded is not None):
    #             meta:DataObjects.MDAudioData2 = DataObjects.MDAudioData2._make(decoded[1])
    #             srate = int(1/meta.dt)
    #             return dict({'channels': 1,
    #                         'sample_rate': srate,
    #                         'bitrate': 16 * srate})
    #     except Empty:
    #         return None
    #     except Exception:
    #         return None
    
    ''' Returns the Spectrum frame '''
    def getSpectrum(self):
        with self.spec_semaphore:
            return (self.scamUtils.p_getFrequencies(), 
                    self.scamUtils.getSpectrumBuffer()[-self.scamUtils.energy_sz:])
    
    ''' Returns the current blob data '''
    def getBlobData(self):
        return self.scamUtils.p_getBlobData()
    
    ''' Returns the signal analysis in the Spectrum 
        NamedTuple: Mean Energy, Std Dev, High Energy Thresh, Current Energy, Low Energy Thresh, SNR, 
        pre-activation flag, detection flag
    '''
    def getSignalInfo(self):
        return self.signalInfo
    
    ''' Sets & Returns the scaling Mode for the Acoustic filter '''
    def setScalingMode(self, mode, max=None, dynamic=5.0, crest=3.1):
        #configure acoustic filter
        self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                    max=max, crest=crest)
        return True
    
    def getScalingMode(self):
        return self.scamUtils.getScalingMode()
    
    def isContinuousStream(self):
        return self.hasStreamData

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
        if(frame is None):
            return None
        truth_tbl = np.where(frame[:,:] == 0.0, True, False)
        frame *= 255
        try:
            frame = frame.astype('uint8')
        except Exception as e:
            pass
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

    signal.signal(signal.SIGINT, camObj.signal_handler)
    #atexit.register(camObj.release_shared)
    #atexit.register(camObj.disconnect)

    if(camObj.connect()):
        print('Sending Initial configuration and preparing state')
        camObj.initialConfigure()
        while(not camObj.hasStatus()):
            pass
        print(camObj.getStatus(), '\n')

    #camObj.restartCamera()
    #time.sleep(3.0)

    camObj.startMeasurement()
    try:
        while(camObj.processData):
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Stopping all processes...")
        camObj.disconnect()