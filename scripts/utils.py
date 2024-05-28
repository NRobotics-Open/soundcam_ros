import numpy as np
from acoustics.decibel import dbmul, dbmean, dbsum, dbdiv
import matplotlib.pyplot as plt
import librosa, wave
import time
from multiprocessing import shared_memory
import SharedArray as sa
import pickle
from enum import Enum

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
    
    '''
        Initializes the shared memory buffers
    '''
    def initializeSharedBuffers(self, semls, shmnamesls:list):
        self.shmnames = shmnamesls
        if(len(self.shmnames) < 3):
            print('Sharedmem Buffer name length mismatch!')
            exit(-99)
        self.sems = semls
        
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
            self.temporalBuffer = np.ndarray((self.acslength, 1), dtype=np.float32)
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
            self.sems[self.BufferTypes.AUDIO.value].acquire()
            newaudioBuffer = np.roll(self.audioBuffer, -shift, axis=0)
            newaudioBuffer[-shift:] = buffer
            self.audioBuffer[: , :] = newaudioBuffer[:, :]
            self.sems[self.BufferTypes.AUDIO.value].release()
        except Exception as ex:
            print(f"Exception occured: {ex}")
    
    '''
        Updates the ringbuffer -> Spectrum Stream
    '''
    def updateSpectrumBuffer(self, data:list):
        self.sems[self.BufferTypes.GLOBAL_SPECTRUM.value].acquire()
        newGBuffer = np.roll(self.specGlobalBuffer, -1, axis=0)
        newGBuffer[-1, : ] = data[2][ : ]
        self.specGlobalBuffer[:, :] = newGBuffer[:, :]
        self.sems[self.BufferTypes.GLOBAL_SPECTRUM.value].release()

        self.sems[self.BufferTypes.LOCAL_SPECTRUM.value].acquire()
        newLBuffer = np.roll(self.specLocalBuffer, -1, axis=0)
        newLBuffer[-1, :] = data[3][ : ]
        self.specLocalBuffer[:, :] = newLBuffer[:, :]
        self.sems[self.BufferTypes.LOCAL_SPECTRUM.value].release()

        self.computeEnergy()

        self.cnt += 1
        if(self.cnt > self.window):
            self.cnt = self.window
    
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

        self.sems[self.BufferTypes.ACOUSTIC.value].acquire()
        newacBuffer = np.roll(self.acousticBuffer, -3072, axis=0)
        if(self.useRunningMax):
            self.maxScale = np.max(data[1])
            if(self.scalingMode == self.ScalingMode.SMART):
                self.maxScale = self.crestVal + np.average(data[1])
            self.minScale = self.maxScale - self.dynScale
        dt = data[1].flatten()
        dt.shape = -1, 1
        #push raw to temporal buffer
        self.temporalBuffer = np.roll(self.temporalBuffer, -3072, axis=0)
        self.temporalBuffer[-3072:] = dt[:]
        dt_norm = (dt - self.minScale)/(self.maxScale - self.minScale)
        dt_norm[np.where(dt_norm < 0.0)] = 0.0
        dt_norm[np.where(dt_norm > 1.0)] = 1.0

        newacBuffer[-3072:] = dt_norm[:]
        self.acousticBuffer[: , :] = newacBuffer[:, :]
        self.sems[self.BufferTypes.ACOUSTIC.value].release()
        self.accnt +=1
        if(self.accnt > self.acWindow):
            self.accnt = self.acWindow
        # print(f"max: {self.maxScale}, min: {self.minScale}, \
        #       dynamic: {self.dynScale}, crestVal: {self.crestVal}")
        return dt_norm.reshape(48, 64) #return a copy
    
    '''
        Sets the scaling mode for acoustic data
    '''
    def setScalingMode(self, mode: ScalingMode, dynamic=3.1, max=None, crest=5):
        self.scalingMode = mode
        self.dynScale = dynamic if(dynamic >= 3.1) else 3.1
        self.maxScale = max
        self.crestVal = 5.0 if(crest < 5.0) else 15.0 if(crest > 15.0) else crest 
        self.modeChange = True
            
    '''
        Calculates the cummulative energy in the given window
    '''
    def computeEnergy(self, uselocal=True)->(float):
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
                    energies[iter.index] = dbsum([energies[iter.index], bin[iter.index]])
        #return (np.mean(energies), np.std(energies))
        self.sems[self.BufferTypes.ENERGY.value].acquire()
        newenergyBuffer = np.roll(self.energyBuffer, -1023, axis=0)
        energies.shape = -1, 1
        newenergyBuffer[-1023:] = energies[:]
        self.energyBuffer[: , :] = newenergyBuffer[:, :]
        self.sems[self.BufferTypes.ENERGY.value].release()
        return energies

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
            self.sems[2].acquire()
            arr = self.specLocalBuffer
            self.sems[2].release()
        else:
            self.sems[1].acquire()
            arr = self.specGlobalBuffer
            self.sems[1].release()
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
        print('DB Sum: ', dbsum(data))
        print('Mean: ', dbmean(data))
    
    def writeOutVar(self, filename:str, uselocal = True):
        with open(filename + '.pkl', 'wb') as f:
            if(uselocal):
                buffer = self.specLocalBuffer[-self.cnt : , :]
                pickle.dump(buffer, f)
            else:
                buffer = self.specGlobalBuffer[-self.cnt : , :]
                pickle.dump(buffer, f)
            print('Saved to file!')
