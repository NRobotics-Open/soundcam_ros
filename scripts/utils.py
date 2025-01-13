import numpy as np
import matplotlib.pyplot as plt
import wave, time, math, pickle
import SharedArray as sa
from enum import Enum
import cv2
from typing import NamedTuple

NFFT = 4096
NOVERLAP = 128

SignalInfo = NamedTuple('SignalInfo', 
                        [('mean_energy', float), ('std_dev', float),
                         ('hi_thresh', float), ('current_energy', float), 
                         ('lo_thresh', float), ('acoustic_energy', float), 
                         ('snr', float), ('pre_activation', bool), 
                         ('detection', bool)])

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

'''
-----------------------------------------EUCLIDEAN DISTANCE TRACKER
'''
from typing import List
BlobInfo = NamedTuple('BlobInfo', 
                        [('id', float), 
                         ('x', float), ('y', float),
                         ('width', float), ('height', float),
                         ('cx', float), ('cy', float),
                         ('Area', float)])
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
    
    def filterDetectionsByID(self, blob_data: List[BlobInfo]) -> List[BlobInfo]:
        # Use a dictionary to keep track of the largest blob for each ID
        largest_blobs = {}

        for blob in blob_data:
            # If the ID is not in the dictionary or if the new blob has a larger area, update the dictionary
            if blob.id not in largest_blobs or blob.Area > largest_blobs[blob.id].Area:
                largest_blobs[blob.id] = blob

        # Return the largest blob for each unique ID
        return list(largest_blobs.values())

    def update(self, objects_rect:list):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for obj in objects_rect:
            blob = BlobInfo(*obj)._asdict()
            blob['cx'] = (blob['x'] + blob['x'] + blob['width']) // 2
            blob['cy'] = (blob['y'] + blob['y'] + blob['height']) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(blob['cx'] - pt[0], blob['cy'] - pt[1])

                #print('distance: ', dist)
                if dist < 200:
                    self.center_points[id] = (blob['cx'], blob['cy'])
                    # print('Center points: ', self.center_points)
                    # print('Dist: ', dist)
                    #objects_bbs_ids.append([x, y, w, h, id, A])
                    blob['id'] = id
                    objects_bbs_ids.append(BlobInfo(**blob))
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if not same_object_detected:
                self.center_points[self.id_count] = (blob['cx'], blob['cy'])
                blob['id'] = self.id_count
                objects_bbs_ids.append(BlobInfo(**blob))
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            center = self.center_points[obj_bb_id.id]
            new_center_points[obj_bb_id.id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids
    
    def detection(self, frame, track=True):
        if(self.prev_frame is None):
            self.prev_frame = frame.copy()
            return []
        #diff = cv2.absdiff(self.prev_frame, frame)  # this method is used to find the difference bw two  frames
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0 )

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
            detections.append(BlobInfo(0, x, y, w, h, 0, 0, contourArea))
        self.prev_frame = frame
        detections = self.filterDetectionsByID(detections)
        if(track):
            # object tracking 
            return self.update(detections)
        return detections
    
    def drawDetections(self, blob_data: list, frame: np.array, getResult=False):
        # Ensure frame is valid
        if frame is None or len(frame.shape) != 3:
            return

        if len(blob_data) > 0:
            for blob in blob_data:
                # Convert coordinates to integers
                x, y, width, height = int(blob.x), int(blob.y), int(blob.width), int(blob.height)

                # Draw blob ID
                cv2.putText(frame, str(blob.id), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                # Draw rectangle
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 1)

            if getResult:
                return frame

            # Show the image
            cv2.imshow('frame', frame)
            cv2.waitKey(1)  # Ensure window refreshes
        else:
            return frame
