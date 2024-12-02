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

SignalInfo = namedtuple('SignalInfo', 
                        'mean std_dev hi_thresh current lo_thresh acoustic SNR pre_activation detection')

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
                 minFreq=0, maxFreq=1e5, acFreq=50, specFreq=11,
                 scalingMode:ScalingMode=ScalingMode.OFF, 
                 hi_thresh_f=1,
                 low_thresh_f=0.5,
                 trigger_thresh=3,
                 smoothing_win=5,
                 trigger_duration=2.0, debug=False) -> None:
        self.debug = debug
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
        self.speclen = int(self.ts * 0.5 * specFreq * 1023)
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

        #SIGNAL Detection
        '''
        - Frequency content filtering
        - Signal smoothing over x-window samples
        - Dynamic Thresholding
        - Hysteresis
        - Temporal Consistency
        '''
        self.hi_threshold_factor = hi_thresh_f #1.5 Multiplier for dynamic threshold
        self.low_threshold_factor = low_thresh_f # Lower threshold for hysteresis
        self.previous_detection = False
        self.pre_activation = False
        self.smoothing_window = smoothing_win  # Sliding window size for energy smoothing
        self.trigger_time = time.time()
        self.trigger_duration = trigger_duration #seconds
        self.trigger_thresh = trigger_thresh
        self.energy_sz = 1023 - self.smoothing_window + 1
        self.energy_skip = self.smoothing_window * self.energy_sz
        self.sig_data:SignalInfo = SignalInfo(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False)
        self.isReady = False

    '''
        Class member variable getters
    '''
    def p_getWindow(self):
        return self.window
    
    def p_getAudLen(self):
        return self.audlength
    
    def p_getACLen(self):
        return self.acslength
    
    def p_getSpecLen(self):
        return self.speclen
    
    def p_getFrequencies(self):
        return self.frequencies
    
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
            self.noiseBuffer = np.zeros((self.p_getSpecLen(), 1))
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
            #focuse audio
            buffer = (data[2] + data[3]).reshape(-1, 1) #reshape to (2048, 1)
            self.audioBuffer[:-2048, :] = self.audioBuffer[2048:, :]
            self.audioBuffer[-2048:, :] = buffer
        except Exception as ex:
            print(f"Exception occured: {ex}")
    
    def getAudioBuffer(self):
        return self.audioBuffer
    
    '''
        Updates the ringbuffer -> Spectrum Stream
    '''
    def updateSpectrumBuffer(self, data:list, uselocal=True):
        if(not uselocal):
            self.specGlobalBuffer[:-1, :] = self.specGlobalBuffer[1:, :]
            self.specGlobalBuffer[-1, :] = data[2][:]
        else:
            self.specLocalBuffer[:-1, :] = self.specLocalBuffer[1:, :]
            self.specLocalBuffer[-1, :] = data[3][:]
        self.computeEnergy(uselocal=uselocal)

        self.cnt += 1
        if(self.cnt > self.window):
            self.cnt = self.window
        if(self.cnt > self.detectionWindow * 1):
            self.isReady = True
    
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

        self.acousticBuffer[:-3072, :] = self.acousticBuffer[3072:, :]
        if(self.useRunningMax):
            self.maxScale = np.max(data[1])
            if(self.scalingMode == self.ScalingMode.SMART):
                self.maxScale = self.crestVal + np.average(data[1])
            self.minScale = self.maxScale - self.dynScale
        dt = data[1].reshape(-1, 1)
        try:
            dt_norm = (dt - self.minScale)/(self.maxScale - self.minScale)
            # dt_norm[np.where(dt_norm < 0.0)] = 0.0
            # dt_norm[np.where(dt_norm > 1.0)] = 1.0
            dt_norm = np.clip(dt_norm, 0.0, 1.0)
        except RuntimeWarning as e:
            return None

        self.acousticBuffer[-3072:, :] = dt_norm
        # self.accnt +=1
        # if(self.accnt > self.acWindow):
        #     self.accnt = self.acWindow
        return dt_norm.reshape(48, 64) #return a copy
    
    def getAcousticBuffer(self):
        return self.acousticBuffer
    
    '''
        Sets and returns the scaling mode for acoustic data
    '''
    def setScalingMode(self, mode: ScalingMode, dynamic=3.1, max=None, crest=5, minF=0, maxF=1e5):
        self.scalingMode = mode
        self.dynScale = dynamic if(dynamic >= 3.1) else 3.1
        self.maxScale = max
        self.crestVal = 5.0 if(crest < 5.0) else 15.0 if(crest > 15.0) else crest 
        self.minFreq = minF
        self.maxFreq = maxF
        self.modeChange = True

    def getScalingMode(self):
        return (self.scalingMode, self.dynScale, self.maxScale, self.crestVal)
            
    '''
        Calculates the cummulative energy in the given window
    '''
    # def computeEnergy(self, uselocal=True, getArray=True)->(float):
    #     if(uselocal):
    #         arr = self.specLocalBuffer[-self.detectionWindow: , :]
    #     else:
    #         arr = self.specGlobalBuffer[-self.detectionWindow: , :]
        
    #     energies = np.zeros((1023, ), dtype=np.float32)
    #     iter = np.nditer(self.frequencies, flags=['f_index'])
    #     for s in range(arr.shape[0]):#for each time sample
    #         bin = arr[s, : ] # (1023, )
    #         for a in iter: #iterate over the content of bins
    #             if((a >= self.minFreq) and (a <= self.maxFreq)): #filter by freq range
    #                 energies[iter.index] = self.dbsum([energies[iter.index], bin[iter.index]])
    #     #self.sems[self.BufferTypes.ENERGY.value].acquire()
    #     newenergyBuffer = np.roll(self.energyBuffer, -1023, axis=0)
    #     energies.shape = -1, 1
    #     newenergyBuffer[-1023:] = energies[:]
    #     self.energyBuffer[: , :] = newenergyBuffer[:, :]
    #     #self.sems[self.BufferTypes.ENERGY.value].release()
    #     if(getArray):
    #         return energies
    #     return (np.mean(energies), np.std(energies))
    
    def computeEnergy(self, uselocal=True, getArray=True) -> (float):
        windowToUse = self.detectionWindow if(self.cnt >= self.detectionWindow) else self.cnt
        #if (uselocal):
        filt_arr = self.specLocalBuffer[-windowToUse:, :]
        #else:
        unfilt_arr = self.specGlobalBuffer[-windowToUse:, :]
        # Initialize energies to zero
        energies = np.zeros(1023, dtype=np.float32)
        noise = np.zeros(1023, dtype=np.float32)
        # Compute the energy for each time sample
        for s in range(filt_arr.shape[0]):  # iterate over each time sample
            filt_bin = filt_arr[s, :]  # Get the frequency bin for the current time sample (shape: 1023,)
            unfilt_bin = unfilt_arr[s, :]
            #print('Bin contains: ', bin)
            # Iterate over each frequency and apply the frequency mask
            for i, freq in enumerate(self.frequencies):
                if self.minFreq <= freq <= self.maxFreq:  # Only consider frequencies within the specified range
                    energies[i] = self.dbsum([energies[i], filt_bin[i]])
                else:
                    noise[i] = self.dbsum([noise[i], unfilt_bin[i]])
            #print('Bin at ',s, ' Energy: ', np.sum(energies))
        # Shift the energy buffer in-place (without np.roll) 
        self.energyBuffer[:-self.energy_sz, :] = self.energyBuffer[self.energy_sz:, :]
        energies = energies.reshape(-1, 1)
        self.energyBuffer[-self.energy_sz:, :] = self.smoothEnergy(energies).reshape(-1,1)
        
        self.noiseBuffer[:-self.energy_sz, :] = self.noiseBuffer[self.energy_sz:, :]
        noise = noise.reshape(-1, 1)
        self.noiseBuffer[-self.energy_sz:, :] = self.smoothEnergy(noise).reshape(-1,1)
        if(self.isReady):
            self.sig_data = self._detectEvent(self.energyBuffer, self.noiseBuffer)
    
    def smoothEnergy(self, energies): # Simple moving average for smoothing returns array of shape (1019,)
        return np.convolve(np.ravel(energies), np.ones(self.smoothing_window)/self.smoothing_window, mode='valid')
    
    
    '''
        Compute the dynamic thresholds w SNR 
    '''
    def dynamicThreshold(self, mean_energy, std_energy, snr):
        # Dynamic threshold calculation based on mean + k * std
        ajusted_factor = self.hi_threshold_factor/ (1 + (snr))
        return (mean_energy + (ajusted_factor * std_energy), 
                self.low_threshold_factor * std_energy)
    
    '''
        Compute the dynamic thresholds w/o SNR 
    '''
    def dynamicThreshold2(self, mean_energy, std_energy, snr, use_snr=False):
        # Dynamic high threshold calculation based on mean + k * std 
        # Dynamic low threshold calculation based on mean + h * std 
        # where k > h
        if(use_snr):
            if(math.isnan(snr)):
                snr = 0.0
            adj_hi = self.hi_threshold_factor/ (1 + (snr))
            adj_lo = self.low_threshold_factor/ (1 + (snr))
            return (mean_energy + (adj_hi * std_energy), 
                mean_energy + (adj_lo * std_energy))
        return (mean_energy + (self.hi_threshold_factor * std_energy), 
                mean_energy + (self.low_threshold_factor * std_energy))
    
    def _detectEvent(self, energies, noise):
        # Step 2: Smooth the energy using a sliding window
        #smoothed_energies = self.smoothEnergy(energies)
        #print("Smoothed Energies | ", smoothed_energies)
        # Step 3: Compute mean and standard deviation of the smoothed energy for dynamic thresholding
         # What if I preclude the amount of input signals I am using for smoothing when calculating the mean and std_dev
        mean_energy = np.nanmean(energies)
        std_energy = np.nanstd(energies)
        #mean_energy = np.mean(energies[: -self.energy_skip])
        #std_energy = np.std(energies[: -self.energy_skip])
        # Step 4: Detect if current energy exceeds the high threshold (new detection) or is below the low threshold (end of detection)
        current_energy = np.nanmean(np.trim_zeros(energies[-self.energy_sz:]))  # Most recent energy value
        # Step 5: Compute SNR 
        # noise_energy = np.mean(energies[:-self.energy_sz])
        # #noise_energy = np.mean(energies[:-self.energy_skip]) #don't use the x-recent signals in the calculation
        # if(noise_energy == 0):
        #     noise_energy = np.inf
        # snr = current_energy/ noise_energy
        # SNR compute with acoustic data
        # ac_avg = np.nanmean(self.acousticBuffer[:-3072])
        # if((ac_avg == 0) or math.isnan(ac_avg)):
        #     ac_avg = np.inf
        # ac_snr = np.nanmean(self.acousticBuffer[-3072:])/ ac_avg
        # if(math.isnan(ac_snr)):
        #     ac_snr = 0.0
        ac_energy = np.sum(self.acousticBuffer[-3072:])
        # print("Ac SNR --- ", ac_snr)
        # if(ac_energy != 0.0):
        #     print(f"acoustic-energy = {ac_energy}")

        # Step 5: Compute SNR w Noise 
        noise_mean = np.nanmean(noise)
        if(noise_mean == 0):
            noise_mean = np.inf
        snr = current_energy/ noise_mean
        #print("Energy SNR %f : NOISE SNR %f : Mean %f" %(snr, noise_mean))
        
        # Step 6: Compute the dynamic threshold and apply hysteresis logic
        #high_threshold, low_threshold = self.dynamicThreshold(mean_energy, std_energy, snr)
        high_threshold, low_threshold = self.dynamicThreshold2(mean_energy, std_energy, snr, use_snr=True)

        #if(self.debug):
        
        
        if self.previous_detection:
            # We are currently detecting, check if energy falls below low threshold
            if ((ac_energy < self.trigger_thresh) and (current_energy < low_threshold)):
                self.previous_detection = False  # Reset detection
                self.pre_activation = False
                self.timer_set = False
                if(self.debug):
                    print("\n\n--------------------------------Resetting detection!")
        else:
            # Check if current energy exceeds the high threshold to trigger a new detection
            if ((ac_energy >= self.trigger_thresh) or (current_energy > high_threshold)):
                self.pre_activation = True
                self.elapsed_t = time.time()-self.trigger_time
                #print("Elapsed: ", self.elapsed_t)
                if(self.elapsed_t >= self.trigger_duration):
                    self.previous_detection = True
                    if(self.debug):
                        print("\n\n++++++++++++++++++++++++++++++++Triggering detection!")
            else: 
                self.trigger_time = time.time()
                self.pre_activation = False
        return SignalInfo(mean_energy, std_energy, high_threshold, current_energy, low_threshold, 
                          ac_energy, snr, self.pre_activation, self.previous_detection)
    
    def getSignalAnalysis(self):
        return self.sig_data

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
        self.cnt = 0
        self.isReady = False
    
    def dbsum(self, levels, axis=None):
        #print('DBSum Levels: ', levels)
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
                 'setScalingMode', 'p_getWindow', 'p_getAudLen', 'p_getACLen', 'p_getSpecLen', \
                 'p_getFrequencies', 'initializeSharedBuffers', 'getSignalAnalysis', \
                 'writeOutVar', 'writeAudio', 'updateSpectogramPlot', 'canRunAnalysis', \
                 'initSpectogram', 'updateAcousticBuffer', 'updateAudioBuffer', \
                 'updateSpectrumBuffer', 'p_getBlobData', 'updateBlobData', \
                 'getScalingMode', 'resetBuffers')
    
    #-------------------------------------------parameter accessors
    def p_getWindow(self):
        return self._callmethod('p_getWindow')
    
    def p_getAudLen(self):
        return self._callmethod('p_getAudLen')

    def p_getACLen(self):
        return self._callmethod('p_getACLen')
    
    def p_getSpecLen(self):
        return self._callmethod('p_getSpecLen')
    
    def p_getFrequencies(self):
        return self._callmethod('p_getFrequencies')
    
    def p_getBlobData(self):
        return self._callmethod('p_getBlobData')
    
    #-----------------------------------------------operation methods
    def initializeSharedBuffers(self, shmnamesls:list):
        return self._callmethod('initializeSharedBuffers', (shmnamesls,))
    
    def getSignalAnalysis(self):
        return self._callmethod('getSignalAnalysis')
    
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
    
    def setScalingMode(self, mode: SoundUtils.ScalingMode, dynamic=3.1, max=None, crest=5, minF=0, maxF=1e5):
        return self._callmethod('setScalingMode', (mode, dynamic, max, crest, minF, maxF))
    
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
from dataclasses import dataclass
import yaml

@dataclass
class MissionData:
    uuid: str
    id: int
    name: str
    result_dir: str

class ROSLayerUtils(object):
    DataPoint = namedtuple('DataPoint', 'id x y theta media mean_energy std_dev current_energy acoustic_energy snr detection isSolved relevant_image \
                           presetName maximumFrequency minimumFrequency distance crest dynamic maximum')
    def __init__(self, debug=False) -> None:
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
        self.localId = 1
        self.debug = debug

    def prepareDirectory(self, id, name, path=None):
        self.missionID = id
        self.missionName = name
        if((path is not None) and (len(path) > 0) and os.path.exists(path)):
            self.path = path
            self.localId = 1 #reset internal id
        else:
            today = datetime.now().strftime("%Y_%m_%d")
            msn_time = datetime.now().strftime("%H_%M")
            self.path = os.path.join(self.msnDir, id, today, msn_time)
            if(not os.path.exists(self.path)): 
                os.makedirs(self.path)
        self.localId = 1 #reset internal id
    
    def getUniqueName(self, isImg=True, suffix=''):
        today = datetime.now().strftime("%Y_%m_%d_")
        msn_time = datetime.now().strftime("%H_%M_%S")
        if(isImg):
            return ''.join(['IMG_', suffix, '_', today, msn_time, '.jpg'])
        else:
            return ''.join(['VID_', suffix, '_', today, msn_time, '.webm'])
    
    def getPath(self, fetchMsnDir=False):
        if(fetchMsnDir):
            return self.path
        else:
            return self.mediaDir
    
    def addMetaData(self, media, info, id=None, isActionPoint=False, preset=None, loop=1, sigInfo:SignalInfo=None, imgIdx=0, useMsnPath=False):
        try:
            assignedId = self.localId if (id is None) else id
            preset_dt = ('', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            if(preset is not None):
                preset_dt = (preset.presetName, preset.maxFrequency, preset.minFrequency,
                             preset.distance, preset.crest, preset.dynamic, preset.maximum)
            obj:ROSLayerUtils.DataPoint = ROSLayerUtils.DataPoint(assignedId, 
                                                float(info[0]), float(info[1]), float(info[2]), 
                                                media,
                                                float(sigInfo.mean), float(sigInfo.std_dev), 
                                                float(sigInfo.current), float(sigInfo.acoustic),
                                                float(sigInfo.SNR), sigInfo.detection,
                                                False, int(imgIdx), *preset_dt)
            path = self.getPath(fetchMsnDir=useMsnPath)
            loop = str(loop)
            #print('Current Loop is: ', loop)
            if(os.path.exists(os.path.join(path, 'meta-data.yaml'))): #read meta data file
                with open(os.path.join(path, 'meta-data.yaml') , 'r') as infofile:
                    self.metaData = yaml.safe_load(infofile)
                    #check by the current loop
                    if(loop not in self.metaData.keys()):
                        self.metaData[loop] = {'datapoints':[], 'actionpoints':[]}
            else:
                self.metaData = dict({loop: {'datapoints':[], 'actionpoints':[]}})

            hasId = False
            if(isActionPoint):
                for obj_old in self.metaData[loop]['actionpoints']: #check if actionpoint in metadata
                    if(obj_old['id'] == assignedId):
                        hasId = True
                        for dt in obj.media:
                            obj_old['media'].append(dt)
                        break
                if(not hasId):
                    self.metaData[loop]['actionpoints'].append(obj._asdict())
            else:
                for obj_old in self.metaData[loop]['datapoints']: #check if datapoint in metadata
                    #print('Existing content: ', obj_old)
                    if(obj_old['id'] == assignedId):
                        hasId = True
                        for dt in obj.media:
                            obj_old['media'].append(dt)
                        break
                if(not hasId):
                    self.metaData[loop]['datapoints'].append(obj._asdict())
                self.localId += 1

            with open(os.path.join(path, 'meta-data.yaml') , 'w') as infofile: #write meta data file
                yaml.dump(self.metaData, infofile)
            return True
        except Exception as e:
            print('Exception caught! ', e)
            return False

    def eulerDistance(self, coords:list):
        return math.sqrt(math.pow((coords[0] - 320), 2) + 
                         math.pow((coords[1] - 240), 2))
    
    def quaternionToEulerDegrees(self, w, x, y, z):
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
        return np.degrees([roll_x, pitch_y, yaw_z]).tolist()
    
    def calcEuclidDistance(self, pose_a:list, pose_b:list):
        return math.sqrt((math.pow((pose_a[0] - pose_b[0]), 2)) + 
                         (math.pow((pose_a[1] - pose_b[1]), 2)))

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
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        if(self.debug):
            print('Saving snaphsot to: ', save_to)
        cv2.imwrite(save_to, frame)
    
    def createVideoFromFrames(self, frame_list:list, filename=None, fps=10):
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
        fourcc = cv2.VideoWriter_fourcc(*'vp80')  # You can use other codecs like 'XVID'
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curVid)
        else:
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        
        if('THM' in filename):
            fps = 4
        out = cv2.VideoWriter(save_to, fourcc, fps, frame_size)
        
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
    
    '''
    Publishes the frame to the given devstream object
    '''
    def publishDevStream(self, frame):
        # Get the shape of the frames
        layers = None
        try:
            _, _ = frame.shape
        except Exception as e:
            _, _, layers = frame.shape

        # Define the codec and push to VideoWriter object
        if(layers is None):
            proc_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        elif (layers == 4):
            proc_frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
        #strmObj.write(proc_frame)
        return proc_frame

    def createAudioFromFrames(self, audio_frames:list, samplerate, filename=None):
        # Concatenate the list of numpy arrays into a single numpy array
        audio_data = np.concatenate(audio_frames)
        # Save the concatenated audio data to a file
        if(filename is None):
            save_to = os.path.join(self.mediaDir, self.curAud)
        else:
            save_to = os.path.join(self.getPath(fetchMsnDir=True), filename)
        sf.write(save_to, audio_data, samplerate)
    
    def imageOverlay(self, img_arr1:np.array, img_arr2:np.array):
        #print(acFrame.shape)
        img_arr1 = cv2.cvtColor(img_arr1, cv2.COLOR_GRAY2BGRA)
        m2 = img_arr2[:,:,3]

        m2i = cv2.bitwise_not(m2)
        alpha2i = cv2.cvtColor(m2i, cv2.COLOR_GRAY2BGRA)/255.0

        # Perform blending and limit pixel values to 0-255 (convert to 8-bit)
        b1i = cv2.convertScaleAbs(img_arr2*(1-alpha2i) + img_arr1*alpha2i)
        return b1i
