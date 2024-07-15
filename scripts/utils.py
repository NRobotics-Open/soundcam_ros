import numpy as np
import matplotlib.pyplot as plt
import wave, time, math, pickle
from multiprocessing.managers import BaseProxy, BaseManager
import SharedArray as sa
from enum import Enum
import cv2, os
import soundfile as sf
from collections import namedtuple

NFFT = 4096
NOVERLAP = 128

class SoundUtils():
    class BufferTypes(Enum):
        AUDIO = 0
        GLOBAL_SPECTRUM = 1
        LOCAL_SPECTRUM = 2
        ENERGY = 3
        ACOUSTIC = 4
    
    class ScalingMode(Enum):
        OFF = 0
        AUTO = 1
        SMART = 2
        

    def __init__(self, window=500, detectionWin=50, t=10, 
                 minFreq=0, maxFreq=1e5, acFreq=50, 
                 scalingMode:ScalingMode=ScalingMode.OFF) -> None:
        self.ax = None
        self.image = None
        self.isAudioBufferInit = False
        self.isSpectrumBufferInit = False
        self.isAcousticBufferInit = False
        self.sampleFreq = 97.65 #Hz
        self.start_t = time.time()

        self.ts = t
        self.acWindow = self.ts * acFreq
        self.audlength = int(self.ts * self.sampleFreq * 2048)
        self.acslength = int(self.acWindow * 3072)
        self.window = window
        self.minFreq = minFreq
        self.maxFreq = maxFreq
        self.detectionWindow = detectionWin
        self.cnt = 0
        self.accnt = 0

        self.maxScale = None #dB
        self.useRunningMax = True
        self.dynScale = 3.1  #dB
        self.minScale = None
        self.crestVal = 5    #dB
        self.scalingMode = scalingMode
        self.blobData = []
        self.hasDetection = False

    '''
        Class member variable getters
    '''
    def p_getWindow(self):
        return self.window
    
    def p_getAudLen(self):
        return self.audlength
    
    def p_getACLen(self):
        return self.acslength
    
    def p_getFrequencies(self):
        return self.frequencies

    ''' Detection updating and fetching '''
    def p_getDetection(self):
        return self.hasDetection
    def updateDetection(self, flag:bool):
        self.hasDetection = flag
    
    ''' Methods for updating and fetching blob data '''
    def p_getBlobData(self):
        return self.blobData
    def updateBlobData(self, val:list):
        self.blobData = val.copy()
    
    '''
        Initializes the shared memory buffers
    '''
    def initializeSharedBuffers(self, shmnamesls:list):
        self.shmnames = shmnamesls
        if(len(self.shmnames) < 3):
            print('Sharedmem Buffer name length mismatch!')
            exit(-99)
        #self.sems = semls
        
        #Audio buffer
        if(not self.isAudioBufferInit):
            self.maxAmp = 2**15
            self.start_t = time.time()
            self.audioBuffer = sa.attach("shm://" + self.shmnames[self.BufferTypes.AUDIO.value])
            self.isAudioBufferInit = True
            print('Audio Ringbuffer ready!')

        #Spectrum buffers
        if(not self.isSpectrumBufferInit):
            self.specGlobalBuffer = sa.attach("shm://" + self.shmnames[self.BufferTypes.GLOBAL_SPECTRUM.value])
            self.specLocalBuffer = sa.attach("shm://" + self.shmnames[self.BufferTypes.LOCAL_SPECTRUM.value])
            self.energyBuffer = sa.attach("shm://" + self.shmnames[self.BufferTypes.ENERGY.value])
            self.frequencies = np.fft.rfftfreq(n=2048, d=1/200000)[2:]
            self.isSpectrumBufferInit = True
            print('Spectrum Ringbuffers ready!')

        #Acoustic buffers
        if(not self.isAcousticBufferInit):
            self.acousticBuffer = sa.attach("shm://" + self.shmnames[self.BufferTypes.ACOUSTIC.value])
            #self.temporalBuffer = np.ndarray((self.acslength, 1), dtype=np.float32)
            self.isAcousticBufferInit = True
            print('Acoustic Ringbuffer ready!')

        print(f"Successfully attached memory buffers!")
    
    '''
        Updates the amplitudes ringbuffer -> Audio Stream
    '''
    def updateAudioBuffer(self, data:list):
        try:
            buffer:np.array = (data[2] + data[3]) #focused audio
            buffer.shape = -1, 1 #reshape to (2048, 1)
            shift = len(buffer)
            #self.sems[self.BufferTypes.AUDIO.value].acquire()
            newaudioBuffer = np.roll(self.audioBuffer, -shift, axis=0)
            newaudioBuffer[-shift:] = buffer
            self.audioBuffer[: , :] = newaudioBuffer[:, :]
            #self.sems[self.BufferTypes.AUDIO.value].release()
        except Exception as ex:
            print(f"Exception occured: {ex}")
    
    def getAudioBuffer(self):
        return self.audioBuffer
    
    '''
        Updates the ringbuffer -> Spectrum Stream
    '''
    def updateSpectrumBuffer(self, data:list):
        #self.sems[self.BufferTypes.GLOBAL_SPECTRUM.value].acquire()
        newGBuffer = np.roll(self.specGlobalBuffer, -1, axis=0)
        newGBuffer[-1, : ] = data[2][ : ]
        self.specGlobalBuffer[:, :] = newGBuffer[:, :]
        #self.sems[self.BufferTypes.GLOBAL_SPECTRUM.value].release()

        #self.sems[self.BufferTypes.LOCAL_SPECTRUM.value].acquire()
        newLBuffer = np.roll(self.specLocalBuffer, -1, axis=0)
        newLBuffer[-1, :] = data[3][ : ]
        self.specLocalBuffer[:, :] = newLBuffer[:, :]
        #self.sems[self.BufferTypes.LOCAL_SPECTRUM.value].release()

        self.computeEnergy()

        self.cnt += 1
        if(self.cnt > self.window):
            self.cnt = self.window
    
    def getSpectrumBuffer(self, global_data=True):
        if(global_data):
            return self.specGlobalBuffer
        return self.specLocalBuffer
    
    '''
        Updates the ringbuffer -> Acoustic Stream
    '''
    def updateAcousticBuffer(self, data:list):
        if(self.modeChange):
            if(self.scalingMode == self.ScalingMode.OFF):
                if((not self.maxScale)):
                    self.useRunningMax = True
                else:
                    self.useRunningMax = False
                    self.minScale = self.maxScale - self.dynScale
            elif(self.scalingMode == self.ScalingMode.AUTO):
                self.useRunningMax = True
            elif(self.scalingMode == self.ScalingMode.SMART):
                self.useRunningMax = True
                if(self.crestVal <= self.dynScale):
                    self.crestVal = (self.crestVal - self.dynScale) + 3.1
            self.modeChange = False

        #self.sems[self.BufferTypes.ACOUSTIC.value].acquire()
        newacBuffer = np.roll(self.acousticBuffer, -3072, axis=0)
        if(self.useRunningMax):
            self.maxScale = np.max(data[1])
            if(self.scalingMode == self.ScalingMode.SMART):
                self.maxScale = self.crestVal + np.average(data[1])
            self.minScale = self.maxScale - self.dynScale
        dt = data[1].flatten()
        dt.shape = -1, 1
        #push raw to temporal buffer
        # self.temporalBuffer = np.roll(self.temporalBuffer, -3072, axis=0)
        # self.temporalBuffer[-3072:] = dt[:]
        try:
            dt_norm = (dt - self.minScale)/(self.maxScale - self.minScale)
            dt_norm[np.where(dt_norm < 0.0)] = 0.0
            dt_norm[np.where(dt_norm > 1.0)] = 1.0
        except RuntimeWarning as e:
            return None

        newacBuffer[-3072:] = dt_norm[:]
        self.acousticBuffer[: , :] = newacBuffer[:, :]
        #self.sems[self.BufferTypes.ACOUSTIC.value].release()
        self.accnt +=1
        if(self.accnt > self.acWindow):
            self.accnt = self.acWindow
        # print(f"max: {self.maxScale}, min: {self.minScale}, \
        #       dynamic: {self.dynScale}, crestVal: {self.crestVal}")
        return dt_norm.reshape(48, 64) #return a copy
    
    def getAcousticBuffer(self):
        return self.acousticBuffer
    
    '''
        Sets and returns the scaling mode for acoustic data
    '''
    def setScalingMode(self, mode: ScalingMode, dynamic=3.1, max=None, crest=5):
        self.scalingMode = mode
        self.dynScale = dynamic if(dynamic >= 3.1) else 3.1
        self.maxScale = max
        self.crestVal = 5.0 if(crest < 5.0) else 15.0 if(crest > 15.0) else crest 
        self.modeChange = True

    def getScalingMode(self):
        return (self.scalingMode, self.dynScale, self.maxScale, self.crestVal)
            
    '''
        Calculates the cummulative energy in the given window
    '''
    def computeEnergy(self, uselocal=True, getArray=True)->(float):
        if(uselocal):
            arr = self.specLocalBuffer[-self.detectionWindow: , :]
        else:
            arr = self.specGlobalBuffer[-self.detectionWindow: , :]
        
        energies = np.zeros((1023, ), dtype=np.float32)
        iter = np.nditer(self.frequencies, flags=['f_index'])
        for s in range(arr.shape[0]):#for each time sample
            bin = arr[s, : ] # (1023, )
            for a in iter: #iterate over the content of bins
                if((a >= self.minFreq) and (a <= self.maxFreq)): #filter by freq range
                    energies[iter.index] = self.dbsum([energies[iter.index], bin[iter.index]])
        #self.sems[self.BufferTypes.ENERGY.value].acquire()
        newenergyBuffer = np.roll(self.energyBuffer, -1023, axis=0)
        energies.shape = -1, 1
        newenergyBuffer[-1023:] = energies[:]
        self.energyBuffer[: , :] = newenergyBuffer[:, :]
        #self.sems[self.BufferTypes.ENERGY.value].release()
        if(getArray):
            return energies
        return (np.mean(energies), np.std(energies))

    '''Check sufficient time passed to initialize analysis'''
    def canRunAnalysis(self):
        return (time.time() - self.start_t) > (self.ts + 1.5)

    def initSpectogram(self, ax:plt):
        self.ax = ax
        extent = [0, self.ts, self.minFreq, self.maxFreq]
        self.image = self.ax.imshow(self.specGlobalBuffer.transpose(), animated=True, origin='lower', extent=extent, aspect='auto', cmap='jet')
        return self.image
    
    def updateSpectogramPlot(self, uselocal=True):
        if(uselocal):
            #self.sems[2].acquire()
            arr = self.specLocalBuffer
            #self.sems[2].release()
        else:
            #self.sems[1].acquire()
            arr = self.specGlobalBuffer
            #self.sems[1].release()
        self.image.set_array(arr.transpose())
        return self.image
    
    def writeAudio(self, filename, data, sr):
        with wave.open(filename, 'w') as wf:
            # Set the parameters
            wf.setnchannels(1)  # Mono channel
            wf.setsampwidth(4)  # 2 bytes per sample (int16)
            wf.setframerate(sr)  # Sampling frequency
            wf.writeframes(data.tobytes())  # Write the data as bytes
    
    def printMetrics(self, data:np.array):
        print('Metrics')
        print('DB Sum: ', self.dbsum(data))
        print('Mean: ', self.dbmean(data))
    
    def writeOutVar(self, filename:str, uselocal=True):
        with open(filename + '.pkl', 'wb') as f:
            if(uselocal):
                buffer = self.specLocalBuffer[-self.cnt : , :]
                pickle.dump(buffer, f)
            else:
                buffer = self.specGlobalBuffer[-self.cnt : , :]
                pickle.dump(buffer, f)
            print('Saved to file!')
    
    def resetBuffers(self):
        self.acousticBuffer[:, :] = 0.0
        self.specLocalBuffer[:, :] = 0.0
        self.specGlobalBuffer[:, :] = 0.0
        self.energyBuffer[:, :] = 0.0
    
    def dbsum(self, levels, axis=None):
        """Energetic summation of levels.

        :param levels: Sequence of levels.
        :param axis: Axis over which to perform the operation.

        .. math:: L_{sum} = 10 \\log_{10}{\\sum_{i=0}^n{10^{L/10}}}

        """
        levels = np.asanyarray(levels)
        return 10.0 * np.log10((10.0**(levels / 10.0)).sum(axis=axis))

    def dbmean(self, levels, axis=None):
        """Energetic average of levels.

        :param levels: Sequence of levels.
        :param axis: Axis over which to perform the operation.

        .. math:: L_{mean} = 10 \\log_{10}{\\frac{1}{n}\\sum_{i=0}^n{10^{L/10}}}

        """
        levels = np.asanyarray(levels)
        return 10.0 * np.log10((10.0**(levels / 10.0)).mean(axis=axis))


class SoundUtilsProxy(BaseProxy):
    _exposed_ = ('getAudioBuffer', 'getSpectrumBuffer', 'getAcousticBuffer', \
                 'setScalingMode', 'p_getWindow', 'p_getAudLen', 'p_getACLen',\
                 'p_getFrequencies', 'initializeSharedBuffers', 'computeEnergy', \
                 'writeOutVar', 'writeAudio', 'updateSpectogramPlot', 'canRunAnalysis', \
                 'initSpectogram', 'updateAcousticBuffer', 'updateAudioBuffer', \
                 'updateSpectrumBuffer', 'p_getBlobData', 'updateBlobData', \
                 'p_getDetection', 'updateDetection', 'getScalingMode', 'resetBuffers')
    
    #-------------------------------------------parameter accessors
    def p_getWindow(self):
        return self._callmethod('p_getWindow')
    
    def p_getAudLen(self):
        return self._callmethod('p_getAudLen')

    def p_getACLen(self):
        return self._callmethod('p_getACLen')
    
    def p_getFrequencies(self):
        return self._callmethod('p_getFrequencies')
    
    def p_getBlobData(self):
        return self._callmethod('p_getBlobData')
    
    def p_getDetection(self):
        return self._callmethod('p_getDetection')
    
    #-----------------------------------------------operation methods
    def initializeSharedBuffers(self, shmnamesls:list):
        return self._callmethod('initializeSharedBuffers', (shmnamesls,))
    
    def computeEnergy(self, uselocal=True, getArray=True):
        return self._callmethod('computeEnergy', (uselocal, getArray,))
    
    def writeOutVar(self, filename:str, uselocal=True):
        return self._callmethod('writeOutVar', (filename, uselocal,))
    
    def writeAudio(self, filename, data, sr):
        return self._callmethod('writeAudio', (filename, data, sr,))
    
    def updateSpectogramPlot(self):
        return self._callmethod('updateSpectogramPlot')
    
    def canRunAnalysis(self):
        return self._callmethod('canRunAnalysis')
    
    def initSpectogram(self, ax:plt):
        return self._callmethod('initSpectogram', (ax,))
    
    def updateAcousticBuffer(self, data:list):
        return self._callmethod('updateAcousticBuffer', (data,))
    
    def updateAudioBuffer(self, data:list):
        return self._callmethod('updateAudioBuffer', (data,))
    
    def updateSpectrumBuffer(self, data:list):
        return self._callmethod('updateSpectrumBuffer', (data,))
    
    #----------------------------------------------------accessor methods
    def getAudioBuffer(self):
        return self._callmethod('getAudioBuffer')
    
    def getSpectrumBuffer(self):
        return self._callmethod('getSpectrumBuffer')
    
    def getAcousticBuffer(self):
        return self._callmethod('getAcousticBuffer')
    
    def setScalingMode(self, mode: SoundUtils.ScalingMode, dynamic=3.1, max=None, crest=5):
        return self._callmethod('setScalingMode', (mode, dynamic, max, crest,))
    
    def getScalingMode(self):
        return self._callmethod('getScalingMode')
    
    def updateBlobData(self, val:list):
        return self._callmethod('updateBlobData', (val,))
    
    def updateDetection(self, flag:bool):
        return self._callmethod('updateDetection', (flag,))
    
    def resetBuffers(self):
        return self._callmethod('resetBuffers')
    

class SoundUtilsManager(BaseManager):
    pass

SoundUtilsManager.register('SoundUtils', SoundUtils, SoundUtilsProxy)


'''
-----------------------------------------EUCLIDEAN DISTANCE TRACKER
'''
class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0
        self.prev_frame = None
    
    def resetIdCount(self):
        self.id_count = 0

    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h, A = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])

                if dist < 50:
                    self.center_points[id] = (cx, cy)
                    # print('Center points: ', self.center_points)
                    # print('Dist: ', dist)
                    objects_bbs_ids.append([x, y, w, h, id, A])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count, A])
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id, _ = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids
    
    def detection(self, frame, track=True):
        if(self.prev_frame is None):
            self.prev_frame = frame.copy()
            return None
        diff = cv2.absdiff(self.prev_frame, frame)  # this method is used to find the difference bw two  frames
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0 )
        # here i would add the region of interest to count the single lane cars 
        # height, width = blur.shape
        # print(height, width)
        

        # thresh_value = cv2.getTrackbarPos('thresh', 'trackbar')
        _, threshold = cv2.threshold(blur, 23, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(threshold, (1,1), iterations=1)
        contours, _, = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        # DRAWING RECTANGLE BOXED
        for contour in contours:
            (x,y,w,h) = cv2.boundingRect(contour)
            contourArea = cv2.contourArea(contour)
            if (contourArea < 1000):
                continue
            #print('Area is: ', contourArea)
            detections.append([x,y,w,h, contourArea])

            # cv2.rectangle(frame1, (x,y),(x+w, y+h), (0,255,0), 2)
            # cv2.putText(frame1, 'status: movement',(10,20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)

        
        # cv2.drawContours(frame1,contours, -1, (0,255,0), 2)
        # cv2.imshow('frame',frame1)
        self.prev_frame = frame
        if(track):
            # object tracking 
            return self.update(detections)
        return detections
    
    def drawDetections(self, boxes_ids:list, frame: np.array, getResult=False):
        for box_id in boxes_ids:
            x,y,w,h,id = box_id
            cv2.putText(frame, str(id),(x,y-15), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.rectangle(frame, (x,y),(x+w, y+h), (0,255,0), 2)
            if(getResult):
                return frame
            cv2.imshow('frame',frame)


'''
-----------------------------------------UTILITY METHODS FOR THE ROS LAYER
'''

from datetime import datetime
import yaml
class ROSLayerUtils(object):
    DataPoint = namedtuple('DataPoint', 'id x y theta media')
    def __init__(self) -> None:
        self.mediaDir = os.path.expanduser("~") + '/current'
        if(not os.path.exists(self.mediaDir)):
            os.makedirs(self.mediaDir)
        self.msnDir = os.path.expanduser("~") + '/missionresults'
        if(not os.path.exists(self.msnDir)):
            os.makedirs(self.msnDir)
        
        self.curImg = 'current.jpg'
        self.curVid = 'current.mp4'
        self.curAud = 'current.wav'
        self.missionID = None
        self.missionName = None
        self.path = None
        self.metaData = dict({'datapoints':[], 'actionpoints':[]})
        self.localId = 1

    def prepareDirectory(self, id, name):
        self.missionID = id
        self.missionName = name
        today = datetime.now().strftime("%Y_%m_%d")
        msn_time = datetime.now().strftime("%H_%m")
        self.path = os.path.join(self.msnDir, id, today, msn_time)
        if(not os.path.exists(self.path)): 
            os.makedirs(self.path)
        self.localId = 1 #reset internal id
    
    def getPath(self):
        if(self.path is not None):
            return self.path
        else:
            return self.mediaDir
    
    def addMetaData(self, media, pose, isActionPoint=False, id=None):
        assignedId = self.localId if (id is None) else id
        obj:ROSLayerUtils.DataPoint = ROSLayerUtils.DataPoint(assignedId, 
                                            pose[0], pose[1], pose[2], media)
        path = self.getPath()
        if(os.path.exists(os.path.join(path, 'meta-data.yaml'))): #read meta data file
            with open(os.path.join(path, 'meta-data.yaml') , 'r') as infofile:
                self.metaData = yaml.safe_load(infofile)
        
        hasId = False
        if(isActionPoint):
            for obj_old in self.metaData['actionpoints']:
                if(obj_old['id'] == assignedId):
                    hasId = True
                    for dt in obj.media:
                        obj_old['media'].append(dt)
                    break
            if(not hasId):
                self.metaData['actionpoints'].append(obj._asdict())
        else:
            for obj_old in self.metaData['datapoints']:
                #print('Existing content: ', obj_old)
                if(obj_old['id'] == assignedId):
                    hasId = True
                    for dt in obj.media:
                        obj_old['media'].append(dt)
                    break
            if(not hasId):
                self.metaData['datapoints'].append(obj._asdict())
            self.localId += 1

        with open(os.path.join(path, 'meta-data.yaml') , 'w') as infofile: #write meta data file
            yaml.dump(self.metaData, infofile)

    def eulerDistance(self, coords:list):
        return math.sqrt(math.pow((coords[0] - 320), 2) + 
                         math.pow((coords[1] - 240), 2))
    
    def quaternionToEulerDegrees(w, x, y, z):
        """
            Convert a quaternion into Euler angles (roll, pitch, yaw) in degrees.

            Parameters:
            w, x, y, z : float
                Quaternion components.

            Returns:
            tuple
                Tuple containing roll, pitch, and yaw in degrees.
        """
        # Convert quaternion to Euler angles in radians
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, +1.0)
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        # Convert radians to degrees
        roll_x = np.degrees(roll_x)
        pitch_y = np.degrees(pitch_y)
        yaw_z = np.degrees(yaw_z)

        return roll_x, pitch_y, yaw_z

    def _calculateMemUsage(self, frame_list:list):
        # Calculate the total memory usage of the frames in Megabytes
        total_bytes = sum(frame.nbytes for frame in frame_list)
        total_megabytes = total_bytes / (1024 * 1024)
        return total_megabytes
    
    def limitMemUsage(self, frame_list:list, max_megabytes):
        # Limit the memory usage of the frames to a given size in Megabytes
        while (self._calculateMemUsage(frame_list) > max_megabytes):
            frame_list.pop(0)  # Remove the oldest frame
    
    def createSnapshotFromFrame(self, frame, filename=None):
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curImg)
        else:
            save_to = os.path.join(self.getPath(), filename)
        print('Saving snaphsot to: ', save_to)
        cv2.imwrite(save_to, frame)
    
    def createVideoFromFrames(self, frame_list:list, path_to_file=None, fps=15):
        if not frame_list:
            raise ValueError("Frame list is empty. Cannot create video.")

        # Get the shape of the frames
        layers = None
        try:
            height, width = frame_list[0].shape
        except Exception as e:
            height, width, layers = frame_list[0].shape
        frame_size = (width, height)

        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can use other codecs like 'XVID'
        if(path_to_file is None):
            out = cv2.VideoWriter(os.path.join(self.mediaDir, self.curVid), 
                                  fourcc, fps, frame_size)
        else:
            out = cv2.VideoWriter(path_to_file, fourcc, fps, frame_size)
        for frame in frame_list:
            #print('Shape: ', frame.shape)
            if(layers is None):
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif (layers == 4):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            out.write(frame)
        # Release everything if job is finished
        out.release()
        frame_list.clear()

    def createAudioFromFrames(self, audio_frames:list, samplerate, output_file=None):
        # Concatenate the list of numpy arrays into a single numpy array
        audio_data = np.concatenate(audio_frames)
        # Save the concatenated audio data to a file
        if(output_file is None):
            sf.write(os.path.join(self.mediaDir, self.curAud), audio_data, samplerate)
        else:
            sf.write(output_file, audio_data, samplerate)
    
    def imageOverlay(self, img_arr1:np.array, img_arr2:np.array):
        #print(acFrame.shape)
        img_arr1 = cv2.cvtColor(img_arr1, cv2.COLOR_GRAY2BGRA)
        m2 = img_arr2[:,:,3]

        m2i = cv2.bitwise_not(m2)
        alpha2i = cv2.cvtColor(m2i, cv2.COLOR_GRAY2BGRA)/255.0

        # Perform blending and limit pixel values to 0-255 (convert to 8-bit)
        b1i = cv2.convertScaleAbs(img_arr2*(1-alpha2i) + img_arr1*alpha2i)
        return b1i
        
