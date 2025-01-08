#!/usr/bin/env python3

'''
This package is developed by N Robotics GmbH to interface with the Sound Camera 
from CAE Sofware & Systems GmbH.

Developer: ephson@nrobotics.com
'''

import socket, time, os, sys, signal, subprocess, select, struct
from enum import Enum
from bitarray import bitarray
import numpy as np, cv2
np.set_printoptions(threshold=sys.maxsize)

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from soundcam_protocol import CommandCodes, DataMessages, MDDataMessage, Device, \
    DataObjects, CameraProtocol, MDLeakRateData, Status
from threading import Thread, Semaphore, Event, Lock
from queue import Empty, Queue
from collections import deque
import SharedArray as sa
import psutil

from utils import SoundUtils as SU, EuclideanDistTracker, SignalInfo


FIRMWARE_VERSION = '2.8.3.0'

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

    def __init__(self, cfgObj:dict={}, debug:bool = False) -> None:
        print('Firmware Version: ', FIRMWARE_VERSION)
        self.debug = debug
        self.testing = False
        self.invokeId = 1
        self.cfgObj = cfgObj

        #debug parameters
        self.new_frame_t = 0
        self.prev_frame_t = 0

        #prepare utils
        self.scamUtils = SU(window=60, detectionWin=5, acFreq=10, specFreq=4, 
                            hi_thresh_f=self.cfgObj['hi_thresh_factor'],
                            low_thresh_f=self.cfgObj['low_thresh_factor'],
                            trigger_thresh=self.cfgObj['trigger_threshold'],
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
        self.processData = Event()
        self.processData.set()
        self.visualUp = False
        self.connected = False
        self.is_alive = False
        self.socket_instances_singleton = False
        self.signalInfo:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        self.leakRate:MDLeakRateData = MDLeakRateData(0, 0, 0, 0, 0)

        #prepare queues
        self.threads = list()
        self.dqueues = list()

        #media Q setup
        self.vidQ = None
        self.acVidQ = None
        self.specQ = None
        self.audQ = None
        self.thermalQ = None

        #post-processing setup
        self.overlayBA = bitarray(self.cfgObj['overlayConfig'])
        self.proc_vidBW = deque(maxlen=60)
        self.proc_vidBW_lock = Lock()
        self.dqueues.append(self.proc_vidBW)
        self.proc_vidOBW = deque(maxlen=60)
        self.proc_vidOBW_lock = Lock()
        self.dqueues.append(self.proc_vidOBW)
        self.proc_acVidQ = deque(maxlen=60)
        self.proc_acVidQ_lock = Lock()
        self.dqueues.append(self.proc_acVidQ)
        self.proc_specQ = deque(maxlen=60)
        self.proc_specQ_lock = Lock()
        self.dqueues.append(self.proc_specQ)
        self.proc_audQ = deque(maxlen=60)
        self.proc_audQ_lock = Lock()
        self.dqueues.append(self.proc_audQ)
        self.proc_thmQ = deque(maxlen=60)
        self.proc_thmQ_lock = Lock()
        self.dqueues.append(self.proc_thmQ)
            
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
            self.vidQ = deque(maxlen=30)
            self.vidQ_lock = Lock()
            self.dqueues.append(self.vidQ)
            self.threads.append(Thread(target=self.processVideo, daemon=True))
        if(self.cfgObj['processAcVideo']):
            self.acVidQ = deque(maxlen=30)
            self.acVidQ_lock = Lock()
            self.dqueues.append(self.acVidQ)
            self.ac_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processAcVideo, daemon=True))
        if(self.cfgObj['processSpectrum']):
            self.specQ = deque(maxlen=30)
            self.specQ_lock = Lock()
            self.dqueues.append(self.specQ)
            self.spec_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processSpectrum, daemon=True))
        if(self.cfgObj['processAudio']):
            self.audQ = deque(maxlen=30)
            self.audQ_lock = Lock()
            self.dqueues.append(self.audQ)
            self.aud_semaphore = Semaphore(1)
            self.threads.append(Thread(target=self.processAudio, daemon=True))
        if(self.cfgObj['processThermal']):
            self.thermalQ = deque(maxlen=30)
            self.thermalQ_lock = Lock()
            self.dqueues.append(self.thermalQ)
            self.threads.append(Thread(target=self.processThermalVideo, daemon=True))
        
        if(self.cfgObj['dataToSend2'] > 0):
            self.leakRateQ = deque(maxlen=30)
            self.leakRateQ_lock = Lock()
            self.dqueues.append(self.leakRateQ)
            self.threads.append(Thread(target=self.processLeakRate, daemon=True))

        # if(self.cfgObj.p_pubRaw):
        #     self.rawQ = Queue()
        #     self.threads.append(Thread(target=self.publishRaw, args=(self.rawQ, self.processData)))
        
        if(self.cfgObj['visualizeAudio'] or self.cfgObj['visualizeSpectrum']):
            self.threads.append(Thread(target=self.displayProcess, args=(self.processData,)))

        #prepare protocol
        self.protocol = CameraProtocol(protocol=self.cfgObj['protocol'], debug=self.debug)
        
        if(self.debug):# Start the memory monitor in the main process
            self.threads.append(Thread(target=self.monitorMemory, daemon=True))
        self.threads.append(Thread(target=self.receiveCyclic, daemon=True))

        #create sockets
        self._createSockets()
        #start threads
        for th in self.threads:
            th.start()
        

        #Note: All data is sent as Little Endian!
    
    '''Ping Camera'''
    def pingCamera(self, deviceIP)->bool:
        proc = subprocess.Popen(["ping", deviceIP, "-c", "1", "-W", "2"], 
                                stdout=subprocess.PIPE)
        proc.wait()
        if(proc.poll() == 0):
            self.is_alive = True
            return True
        return False
    
    '''
        Create socket instances for later use
    '''
    def _createSockets(self):
        if(not self.socket_instances_singleton):
            print('Creating socket instances ...')
            self.ip_addr = self.cfgObj['ip'] 
            self.tcp_port = self.cfgObj['tcp_port']
            self.broadcast_ip = self.cfgObj['broadcast_ip']
            self.udp_port = self.cfgObj['udp_port']
            self.udp_recv_port = self.cfgObj['udp_receive_port']
            
            #UDP sockets creation
            self.udp_sock = socket.socket(socket.AF_INET, # IPv4
                            socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP SOCK_DGRAM   #TCP SOCK_STREAM
            self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            
            self.udp_recv_sock = socket.socket(socket.AF_INET, # IPv4
                            socket.SOCK_DGRAM, socket.IPPROTO_UDP) # UDP SOCK_DGRAM   #TCP SOCK_STREAM
            self.udp_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            if hasattr(socket, "SO_REUSEPORT"):
                self.udp_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.udp_recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_recv_sock.bind(("", self.udp_recv_port))
            self.udp_recv_sock.settimeout(0.5)

            #TCP socket creation
            self.sock = socket.socket(socket.AF_INET, # IPv4
                                    socket.SOCK_STREAM) # UDP SOCK_DGRAM   #TCP SOCK_STREAM
            if hasattr(socket, "SO_REUSEPORT"):
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            self.socket_instances_singleton = True
            print(f'Created IPs: \n\tTCP IP -> {self.ip_addr} \n\tBROADCAST IP -> {self.broadcast_ip} \nPorts: \n\tTCP -> {self.tcp_port} \n\tUDP -> {self.udp_port} \n\tUDP-Recv -> {self.udp_recv_port}')

    
    '''
        Broadcast query for available cameras and status
    '''
    def AKAMsAssemble(self, detectDevice=False):
        try:
            if(not detectDevice):
                MESSAGE = b"SoundCams send your ID"
                self.udp_sock.sendto(MESSAGE, (self.broadcast_ip, self.udp_port))
                try:
                    while True:
                        data, addr = self.udp_recv_sock.recvfrom(1024)  # Buffer size 1024 bytes
                        if(self.debug):
                            print(f"[AKAMs]Id Response (extended) from {addr}: Length {len(data)}") #128 b
                        self.protocol.unpackDecodeResponse(data)
                        
                        if(self.debug):
                            statusObj:Status = self.protocol.getDeviceStatus()['Status']
                            print('StatusObj: ', statusObj)
                        return True
                except socket.timeout:
                    return False
            else:
                MESSAGE = b"Hello AKAMs send your ID"
                self.udp_sock.sendto(MESSAGE, (self.broadcast_ip, self.udp_port))
                self.connected = False
                self.activeTrans = False
                addrs = list()
                try:
                    print('Scanning for device(s) on network ...')
                    while True:
                        data, addr = self.udp_recv_sock.recvfrom(1024)  # Buffer size 1024 bytes
                        
                        if(self.debug):
                            print(f"[AKAMs]Id Response from {addr}: Length {len(data)}") #84 b
                        print(f"[AKAMs]Found device on IP: {addr}")
                        addrs.append(addr)

                        # self.protocol.unpackDecodeResponse(data)
                        # statusObj:Status = self.protocol.getDeviceStatus()['Status']
                except socket.timeout:
                    if(len(addrs) > 0):
                        return addrs
                    else:
                        print("[AKAMs] No device found!")
                        return False
        except Exception as ex:
            print('Uncaught Error in AKAMsAssemble: ', ex)
            return None

    '''
        Establishes connection with the camera and performs handshake (gets device Id)
    '''
    def connect(self):
        conState = False
        try:
            if(self.AKAMsAssemble()): #found connected device
                devStatus = self.protocol.getDeviceStatus()
                #print(f'Connected Device Status: {devStatus}')
                if(devStatus['deviceIP'] != self.ip_addr):
                    print(f"Warning IP Mismatch detected. Connecting to device on: {devStatus['deviceIP']}")
                    self.ip_addr = devStatus['deviceIP']
                if(devStatus['Status'].isListenerCreated == 1):
                    self.sock.connect((self.ip_addr, self.tcp_port))
                    conState = True
            else: #find and connnect to first available device
                print('No Connected devices found, will scan for devices ...')
                res = self.AKAMsAssemble(detectDevice=True)
                if(res is False):
                    return conState
                else:
                    print(f'Attempting connection to -> {res[0]}')
                    self.sock.connect((res[0], self.tcp_port))
                    self.ip_addr = res[0]
        except Exception as ex:
            print('Failure Establishing connection to device! ', ex)
            self.connected = False
            return conState

        print("\nSocket initialized on: \t IP| %s PORT| %d" % (self.ip_addr, self.tcp_port))
        print("Clearing buffer ...")
        self.emptyBuffer()
        print('Fetching device Id ...')
        query = self.protocol.generateReadRequest(command=CommandCodes.IdentificationReq, 
                                                  invokeId=self.invokeId)
        self.invokeId += 1
        query += self.protocol.writeCameraLighting(self.invokeId, 
                                                   self.cfgObj['cameraLighting'])
        self.sendData(query=query)
        return conState

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
        for q in self.dqueues:
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
    
    def _addDequeue(self, q:deque, data:bytes, l: Lock):
        if(q is not None):
            with l:
                q.append(data)

    def disconnect(self):
        if(not self.processData.is_set()):
            return
        
        self.canRun = False
        self.processData.clear()
        if(self.connected):
            try:
                cv2.destroyAllWindows()
                plt.close()
                self.stopMeasurement()
                self.sock.close()
                self.udp_sock.close()
                self.udp_recv_sock.close()
                self.socket_instances_singleton = False
            except Exception as ex:
                print('Error closing connection: ', ex)
        try:
            for q in self.dqueues:
                q.clear()
            print('Joining threads ...')
            for th in self.threads:
                if(th.is_alive()):
                    th.join(timeout=1)
        except Exception as ex:
            print('Error joining processes/threads: ', ex)
    
    ''' Release shared memory blocks '''
    def release_shared(self):
        for i, shm in enumerate(self.shmnames):
            try:
                sa.delete(shm)
            except FileNotFoundError:
                pass
            except Exception:
                pass

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
            #concatenate PrepareState
            self.invokeId += 1
            query += self.protocol.setState(self.invokeId, 
                                            Device.DeviceStates(self.cfgObj['deviceState']))#set device state
            #concatenate DataToSend
            self.invokeId += 1
            query += self.protocol.dataToSendConfig(self.invokeId, 
                                                    dataToSend1=self.cfgObj['dataToSend1'],
                                                    dataToSend2=self.cfgObj['dataToSend2'],
                                                    dataToSend3=self.cfgObj['dataToSend3'])
            self.invokeId += 1
            if(getQuery):
                return query
            self.sendData(query=query)

            while True:
                self.AKAMsAssemble()
                if(self.protocol.getDeviceStatus()['Status'].isConnectedHost == 1):
                    self.connected = True
                    print('Connected to Host!')
                    break
                time.sleep(0.5)

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
    def updatePreset(self, mode:int, distance:int, minFreq:int, maxFreq:int,
                     dynamic:float=3.1, crest:float=5.0, maximum=None):
        #set freq range
        self.invokeId += 1
        query = self.protocol.writeFrequencyRange(self.invokeId, (minFreq, maxFreq))
        self.invokeId += 1
        #self.sendData(query=query)

        #set distance
        query += self.protocol.writeDistance(self.invokeId, distance)
        self.invokeId += 1
        self.sendData(query=query)
        self.scamUtils.setScalingMode(mode=SU.ScalingMode(mode), dynamic=dynamic, 
                                          max=maximum, crest=crest, minF=minFreq, maxF=maxFreq)
        self.scamUtils.resetBuffers()
        return True
    
    def startMeasurement(self)->bool:
        try:
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


    def receiveCyclic(self):
        print('Cyclic thread started ...')
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
                                self._addDequeue(self.vidQ, datablock, self.vidQ_lock)
                            elif(objhdr[0] == DataObjects.Ids.AcousticVideoData.value): #acoustic video
                                self._addDequeue(self.acVidQ, datablock, self.acVidQ_lock)
                            elif(objhdr[0] == DataObjects.Ids.SpectrumData.value): #spectrum
                                self._addDequeue(self.specQ, datablock, self.specQ_lock)
                            elif(objhdr[0] == DataObjects.Ids.AudioData.value): #audio
                                self._addDequeue(self.audQ, datablock, self.audQ_lock)
                            elif(objhdr[0] == DataObjects.Ids.ThermalVideoData.value): #thermal video
                                self._addDequeue(self.thermalQ, datablock, self.thermalQ_lock)
                            elif(objhdr[0] == DataObjects.Ids.CommonStatus.value): #common status
                                #TODO: call function to decode
                                pass
                            elif(objhdr[0] == DataObjects.Ids.LeakRate.value):
                                print('Got Leakage data', datablock)
                                self._addDequeue(self.leakRateQ, datablock, self.leakRateQ_lock)
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

    '''Decodes and Publishes Video data'''
    def processVideo(self):
        print('Starting video processor ...')

        start_t = time.time()
        hits = 0
        acFrame = None
        while(self.processData.is_set()):
            try:
                try:
                    with self.vidQ_lock:
                        raw = self.vidQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                decoded = self.protocol.unpackDecodeVideoData(raw)
                if(self.cfgObj['visualizeVideo']):
                    if(self.cfgObj['visualizeOverlay']):
                        with self.proc_acVidQ_lock:
                            acFrame = self.proc_acVidQ.popleft()
                    self.visualizeBWVideo(decoded[1], acFrame=acFrame)
                else:
                    with self.proc_vidBW_lock:
                        self.proc_vidBW.append(decoded[1])

                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('==================================================================Vid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
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
                try:
                    with self.acVidQ_lock:
                        raw = self.acVidQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                decoded = self.protocol.unpackDecodeAcousticVideoData(raw)
                with self.ac_semaphore:
                    norm0_1 = self.scamUtils.updateAcousticBuffer(decoded)
                    if(self.cfgObj['visualizeAcVideo']):
                        self.visualizeACVideo(norm0_1)

                    res = self.visualizeACVideo(norm0_1, getResult=True)
                    if(res is not None):
                        with self.proc_acVidQ_lock:
                            self.proc_acVidQ.append(res)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('===============================================================AcousticVid @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
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
        hits = 0
        start_t = time.time()
        while(self.processData.is_set()):
            try:
                try:
                    with self.specQ_lock:
                        raw = self.specQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                decoded = self.protocol.unpackDecodeSpectrumData(raw)
                if(decoded is None):
                    continue
                with self.spec_semaphore:
                    self.scamUtils.updateSpectrumBuffer(decoded)
                    self.signalInfo = self.scamUtils.getSignalAnalysis()
                with self.proc_specQ_lock:
                    self.proc_specQ.append(decoded)         
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
            except Exception as e:
                print(f"Error in spectrum process {e}")
                break

    '''Decodes and Publishes Audio data'''
    def processAudio(self):
        print('Starting audio processor ...')

        saveAudio = False
        audioData_plus = []
        audioData_minus = []
        t = 30 #seconds
        record_t = time.time()
        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                try:
                    with self.audQ_lock:
                        raw = self.audQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                decoded = self.protocol.unpackDecodeAudioData(raw)
                with self.aud_semaphore:
                    self.scamUtils.updateAudioBuffer(decoded) #update ringbuffer
                with self.proc_audQ_lock:
                    self.proc_audQ.append(decoded)
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
                        self.scamUtils.writeAudio('outputLong.wav', (audioData_plus + audioData_minus), sr)
                        saveAudio = False
                        
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Audio @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Exception as e:
                print(f"Error in process {e}")
                break


    '''Decodes and Publishes Raw data'''
    def publishRaw(self, q:Queue, canRun:Event):
        print('Starting raw data publisher (for debugging) ...')
        while(canRun.is_set()):
            pass

    '''Decodes and Publishes Thermal Video data'''
    def processThermalVideo(self):
        print('Starting thermal video processor ...')
        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                try:
                    with self.thermalQ_lock:
                        raw = self.thermalQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                decoded = self.protocol.unpackDecodeThermalVideoData(raw)
                if(self.cfgObj['visualizeThermal']):
                    self.visualizeThermal(decoded[1])
                else:
                    res = self.visualizeThermal(decoded[1], getResult=True)
                    with self.proc_thmQ_lock:
                        self.proc_thmQ.append(res)
                
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('================================================================Thermal @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Exception as e:
                print(f"Error in process {e}")
                break
    
    '''Decodes and Publishes Leakage rate data'''
    def processLeakRate(self):
        print('Starting leak rate processor ...')

        start_t = time.time()
        hits = 0
        while(self.processData.is_set()):
            try:
                try:
                    with self.leakRateQ_lock:
                        raw = self.leakRateQ.popleft()
                except IndexError:
                    time.sleep(0.01)
                    continue
                self.leakRate = MDLeakRateData(**self.protocol.unpackDecodeLeakRateData(raw)._asdict())
                print(f"\tLeakRate: {self.leakRate.LeakRate} \t\tStatus: {self.leakRate.State}")
                hits += 1
                if((time.time() - start_t >= 1.0) and self.debug):
                    print('===============================================================LeakRate @ %i Hz' % hits)
                    hits = 0
                    start_t = time.time()
            except Exception as e:
                print(f"Error in leak rate process {e}")
                break
    
    def monitorMemory(self):
        process = psutil.Process(os.getpid())
        while(self.processData.is_set()):
            mem_info = process.memory_info()
            print(f"Memory usage: RSS={mem_info.rss / (1024 * 1024)} MB")
            time.sleep(5)

    '''Updates the plot'''
    def plotUpdater(self, frame):
        if(self.cfgObj['visualizeAudio']):
            try:
                with self.proc_audQ_lock:
                    data = self.proc_audQ.popleft()
                    if(data is not None):
                        audio = data[2] + data[3]
                        lims = np.max(audio) + 20000 # plus buffer range
                        self.ax[0].set_ylim(top=lims, bottom=-lims)
                        self.plotLines[0].set_ydata(audio)
            except IndexError:
                time.sleep(0.01)
        
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
                with self.proc_specQ_lock:
                    data = self.proc_specQ.popleft()
                    if(data is not None):
                        self.plotLines[1].set_ydata(data[2]) #global
                        self.plotLines[2].set_ydata(data[3]) #local
                        # spectogram plotting
                        self.ax_img = self.scamUtils.updateSpectogramPlot()
            except IndexError:
                time.sleep(0.01)
            return self.plotLines, self.ax_img
        return self.plotLines
    

    
    '''Displays live AUDIO/SPECTRUM data'''
    def displayLiveData(self, canRun:Event):
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

    def displayProcess(self, canRun:Event):
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
        self.AKAMsAssemble()
        if(not self.AKAMsAssemble()):
            return False
        return (self.is_alive and 
                (self.protocol.getDeviceStatus()['Status'].isConnectedHost == 1) and 
                (self.protocol.getDeviceStatus()['Status'].isTransferActive == 1))

    ''' Returns the BW Video frame '''
    def getBWVideo(self):
        try:
            with self.proc_vidBW_lock:
                return self.proc_vidBW.popleft()
        except IndexError:
            time.sleep(0.01)
            return None
        except Exception:
            return None
    
    ''' Returns the Overlayed video frame '''
    def getOverlayVideo(self):
        try:
            with self.proc_vidOBW_lock:
                return self.proc_vidOBW.popleft()
        except IndexError:
            time.sleep(0.01)
            return None
        except Exception:
            return None
    
    ''' Returns the Acoustic Video frame '''
    def getACVideo(self):
        try:
            with self.proc_acVidQ_lock:
                return self.proc_acVidQ.popleft()
        except IndexError:
            time.sleep(0.01)
            return None
        except Exception:
            return None
    
    ''' Returns the Thermal Video frame '''
    def getTMVideo(self):
        try:
            with self.proc_thmQ_lock:
                return self.proc_thmQ.popleft()
        except IndexError:
            time.sleep(0.01)
            return None
        except Exception:
            return None
    
    ''' Returns the Audio frame '''
    def getAudio(self):
        with self.aud_semaphore:
            return self.scamUtils.getAudioBuffer()[-2048:]
        
    ''' Returns the Audio Info '''
    def getAudioInfo(self):
        try:
            with self.proc_audQ_lock:
                decoded = self.proc_audQ.popleft()
                if(decoded is not None):
                    meta:DataObjects.MDAudioData2 = DataObjects.MDAudioData2._make(decoded[1])
                    srate = int(1/meta.dt)
                    return dict({'channels': 1,
                                'sample_rate': srate,
                                'bitrate': 16 * srate})
        except IndexError:
            time.sleep(0.01)
            return None
        except Exception:
            return None
    
    ''' Returns the Spectrum frame '''
    def getSpectrum(self):
        with self.spec_semaphore:
            return (self.scamUtils.p_getFrequencies(), self.scamUtils.getSpectrumBuffer()[-1023:])
    
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
    
    def isDetectionReady(self):
        return self.scamUtils.isReady

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

    if(camObj.connect()):
        print('Sending Initial configuration and preparing state')
        camObj.initialConfigure()
        while(not camObj.hasStatus()):
            pass
        print(camObj.getStatus(), '\n')

    camObj.startMeasurement()
    try:
        while(camObj.processData):
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Stopping all processes...")
        camObj.disconnect()