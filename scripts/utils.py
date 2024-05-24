import numpy as np
from acoustics.decibel import dbadd, dbmean, dbsum
import matplotlib.pyplot as plt
import librosa, wave
import time
from multiprocessing import shared_memory
import SharedArray as sa
import pickle

NFFT = 4096
NOVERLAP = 128

class SoundUtils():
    def __init__(self, window=500, detectionWin=50, t=5, 
                 minFreq=0, maxFreq=1e5) -> None:
        self.ax = None
        self.image = None
        self.isAudioBufferInit = False
        self.isSpectrumBufferInit = False
        self.sampleFreq = 97.65 #Hz
        self.start_t = time.time()

        self.ts = t
        self.audlength = int(self.ts * self.sampleFreq * 2048)
        self.window = window
        self.minFreq = minFreq
        self.maxFreq = maxFreq
        self.detectionWindow = detectionWin
        self.cnt = 0
    
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
            self.isAudioBufferInit = True
            print('Audio Ringbuffer ready!')
            self.audioBuffer = sa.attach("shm://" + self.shmnames[0])

        #Spectrum buffers
        if(not self.isSpectrumBufferInit):
            self.specGlobalBuffer = sa.attach("shm://" + self.shmnames[1])
            self.specLocalBuffer = sa.attach("shm://" + self.shmnames[2])
            self.frequencies = np.fft.rfftfreq(n=2048, d=1/200000)[2:]
            self.energies = np.zeros((self.frequencies.shape[0], ), dtype=np.float32)
            self.iter = np.nditer(self.frequencies, flags=['f_index'])
            print('Spectrum Ringbuffers ready!')
            self.isSpectrumBufferInit = True

        print(f"Successfully attached memory buffers!")
    
    '''
        Updates the normalized amplitudes ringbuffer -> Audio Stream
    '''
    def updateAudioBuffer(self, data:list):
        try:
            buffer:np.array = (data[2] + data[3]) #focused audio
            buffer.shape = -1, 1 #reshape to (2048, 1)
            shift = len(buffer)
            self.sems[0].acquire()
            newaudioBuffer = np.roll(self.audioBuffer, -shift, axis=0)
            newaudioBuffer[-shift:] = buffer
            self.audioBuffer[: , :] = newaudioBuffer[:, :]
            self.sems[0].release()
        except Exception as ex:
            print(f"Exception occured: {ex}")
    
    '''
        Updates the normalized (dBA) ringbuffer -> Spectrum Stream
    '''
    def updateSpectrumBuffer(self, data:list):
        
        # if(not self.isSpectrumBufferInit):
        #     #self.specGlobalBuffer = np.ndarray((self.speclength, 1), dtype=np.float32)
        #     self.specGlobalBuffer = np.ndarray((500, 1023), dtype=np.float32)
        #     #self.globalshm = shared_memory.SharedMemory(name=self.shmnames[1], create=True, size=self.specGlobalBuffer.nbytes)
            
        #     #self.specLocalBuffer = np.ndarray((self.speclength, 1), dtype=np.float32)
        #     #self.localshm = shared_memory.SharedMemory(name=self.shmnames[2], create=True, size=self.specLocalBuffer.nbytes)
            
        #     self.start_t = time.time()
        #     self.isSpectrumBufferInit = True
        #     print('Spectrum Ringbuffer up!')
        #reshape to (1024, 1)
        #data[2].shape = -1, 1
        #data[3].shape = -1, 1 
        #shift = len(data[2])
        self.sems[1].acquire()
        newGBuffer = np.roll(self.specGlobalBuffer, -1, axis=0)
        newGBuffer[-1, : ] = data[2][ : ]
        self.specGlobalBuffer[:, :] = newGBuffer[:, :]
        self.sems[1].release()

        self.sems[2].acquire()
        newLBuffer = np.roll(self.specLocalBuffer, -1, axis=0)
        newLBuffer[-1, :] = data[3][ : ]
        self.specLocalBuffer[:, :] = newLBuffer[:, :]
        self.sems[2].release()

        self.cnt += 1
        if(self.cnt > self.window):
            self.cnt = self.window
        #self.sem.acquire()
        #buffer1 = np.ndarray((500, 1023), dtype=np.float32, buffer=self.globalshm.buf)
        #buffer1[:] = self.specGlobalBuffer[:]
        #self.matrix.append(data[2])
        #buffer2 = np.ndarray((self.speclength, 1), dtype=np.float32, buffer=self.localshm.buf)
        #buffer2[:] = self.specLocalBuffer[:]
        #self.sem.release()
    
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
