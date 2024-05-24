#! /usr/bin/env python3

import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)
import matplotlib.pyplot as plt
import librosa, wave
from scipy import signal
from scipy.io import wavfile 
from scipy.signal import find_peaks, correlate
import pickle
from scipy.stats import kurtosis, skew
from acoustics.decibel import dbadd, dbmean, dbsum

wav_file = 'outputSpec_mute.wav'
pkl_file = 'whitenoise_mat_t2.pkl'
#pkl_file = 'file_mat.pkl'
#wav_file = librosa.ex('trumpet')

def graph_spectogram(wav_file):
    y, sr = librosa.load(wav_file, dtype=np.float32) #librosa.ex('trumpet')
    print(y)
    max_amplitude = np.max(np.abs(y))
    y_normalized = y / max_amplitude

    D = librosa.stft(y_normalized)  # STFT of y
    S_db = librosa.amplitude_to_db(np.abs(D), ref=np.max)

    fig, ax = plt.subplots()
    img = librosa.display.specshow(S_db, x_axis='time', y_axis='log', 
                                ax=ax, cmap='jet')
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    #plt.show()

def graph_wavfileread(_wav_file_):
    sample_rate, samples = wavfile.read(_wav_file_)   
    frequencies, times, spectrogram = signal.spectrogram(samples,sample_rate,nfft=1024)
    plt.pcolormesh(times, frequencies, 20*np.log10(spectrogram))
    plt.ylabel('Frequency [Hz]')
    plt.xlabel('Time [sec]')
    #plt.show()
    #plt.savefig("spectogram1.png")

def sound_pressure(audio_file):
    
    y, sr = librosa.load(audio_file, sr=None)

    # Convert the amplitude of the audio signal to sound pressure level (SPL)
    # The following assumes the audio signal 'y' is already normalized between -1 and 1
    # RMS (Root Mean Square) calculation to get the average power of the signal
    # Normalize the audio signal to the range -1 to 1
    max_amplitude = np.max(np.abs(y))
    y_normalized = y / max_amplitude
    rms = librosa.feature.rms(y=y_normalized)[0]

    # Convert RMS to dB SPL
    # Reference value for SPL in air (20 micropascals)
    ref_pressure = 20e-6

    # Compute SPL in dB
    spl_db = 20 * np.log10(rms / ref_pressure)

    # Plot the SPL over time
    frames = range(len(spl_db))
    times = librosa.frames_to_time(frames, sr=sr)

    plt.figure(figsize=(10, 6))
    plt.plot(times, spl_db)
    plt.xlabel('Time (s)')
    plt.ylabel('Sound Pressure Level (dB SPL)')
    plt.title('Sound Pressure Level over Time')
    #plt.show()

def air_leak_detection(wav_file):
    y, sr = librosa.load(wav_file, sr=None)

    # Normalize the audio signal to the range -1 to 1
    max_amplitude = np.max(np.abs(y))
    y_normalized = y / max_amplitude

    # Perform a Short-Time Fourier Transform (STFT)
    D = librosa.stft(y_normalized)

    # Convert the amplitude to decibels
    S_db = librosa.amplitude_to_db(np.abs(D), ref=np.max)

    # Identify the frequencies corresponding to air leaks (typically high frequencies)
    # For this example, we consider frequencies above 5 kHz as potential leak indicators
    frequencies = librosa.fft_frequencies(sr=sr)
    high_freq_indices = np.where(frequencies > 18500)[0]

    # Sum the energy in the high-frequency range
    high_freq_energy = np.sum(S_db[high_freq_indices, :], axis=0)

    # Detect peaks in the high-frequency energy signal
    peaks, _ = find_peaks(high_freq_energy, height=np.mean(high_freq_energy) + np.std(high_freq_energy))

    # Plot the results
    times = librosa.frames_to_time(np.arange(S_db.shape[1]), sr=sr)

    plt.figure(figsize=(14, 6))
    plt.subplot(2, 1, 1)
    librosa.display.specshow(S_db, sr=sr, x_axis='time', y_axis='log')
    plt.colorbar(format='%+2.0f dB')
    plt.title('Spectrogram (dB)')

    plt.subplot(2, 1, 2)
    plt.plot(times, high_freq_energy, label='High Frequency Energy')
    plt.plot(times[peaks], high_freq_energy[peaks], 'x', label='Detected Peaks')
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (dB)')
    plt.title('High Frequency Energy over Time')
    plt.legend()

    plt.tight_layout()
    #plt.show()

    # Output the times of detected peaks
    #print("Detected air leak at times (seconds):", times[peaks])

def air_leak_detection_autocorr(wav_file):
    y, sr = librosa.load(wav_file, sr=None)

    # Normalize the audio signal to the range -1 to 1
    max_amplitude = np.max(np.abs(y))
    y_normalized = y / max_amplitude

    # Perform a Short-Time Fourier Transform (STFT)
    D = librosa.stft(y_normalized)

    # Convert the amplitude to decibels
    S_db = librosa.amplitude_to_db(np.abs(D), ref=np.max)

    # Identify the frequencies corresponding to air leaks (typically high frequencies)
    # For this example, we consider frequencies above 5 kHz as potential leak indicators
    frequencies = librosa.fft_frequencies(sr=sr)
    high_freq_indices = np.where(frequencies > 18500)[0]

    # Sum the energy in the high-frequency range
    high_freq_energy = np.sum(S_db[high_freq_indices, :], axis=0)

    # Perform autocorrelation on the high-frequency energy
    autocorr = correlate(high_freq_energy, high_freq_energy, mode='full')
    autocorr = autocorr[autocorr.size // 2:]  # Take the positive lags

    # Normalize the autocorrelation
    autocorr /= np.max(autocorr)

    # Detect peaks in the autocorrelation signal
    autocorr_peaks, _ = find_peaks(autocorr, height=0.08)  # Adjust the height threshold as needed

    # Detect peaks in the high-frequency energy signal
    high_freq_peaks, _ = find_peaks(high_freq_energy,
                                    height=np.mean(high_freq_energy) + np.std(high_freq_energy))

    # Combine detections: A peak is considered a true detection if it appears in both signals
    combined_peaks = np.intersect1d(high_freq_peaks, autocorr_peaks)

    # Plot the results
    times = librosa.frames_to_time(np.arange(S_db.shape[1]), sr=sr)

    plt.figure(figsize=(14, 8))

    plt.subplot(3, 1, 1)
    librosa.display.specshow(S_db, sr=sr, x_axis='time', y_axis='log')
    plt.colorbar(format='%+2.0f dB')
    plt.title('Spectrogram (dB)')

    plt.subplot(3, 1, 2)
    plt.plot(times, high_freq_energy, label='High Frequency Energy')
    plt.plot(times[high_freq_peaks], high_freq_energy[high_freq_peaks], 'x', label='Detected Peaks')
    plt.xlabel('Time (s)')
    plt.ylabel('Energy (dB)')
    plt.title('High Frequency Energy over Time')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(autocorr, label='Autocorrelation of High Frequency Energy')
    plt.plot(autocorr_peaks, autocorr[autocorr_peaks], 'x', label='Detected Peaks')
    plt.xlabel('Lag')
    plt.ylabel('Autocorrelation')
    plt.title('Autocorrelation of High Frequency Energy')
    plt.legend()

    plt.tight_layout()
    #plt.show()

    # Output the times of detected peaks
    print("Detected air leak HF at times (seconds):", times[high_freq_peaks])
    print("Detected air leak CORR at times (seconds):", times[autocorr_peaks])
    print("Detected air leak at times (seconds):", times[combined_peaks])

def white_noise_detection(wav_file):
    y, sr = librosa.load(wav_file, sr=None)

    # Normalize the audio signal to the range -1 to 1
    max_amplitude = np.max(np.abs(y))
    y_normalized = y / max_amplitude

    # Perform a Short-Time Fourier Transform (STFT)
    D = librosa.stft(y_normalized)

    # Convert the amplitude to power
    S_power = np.abs(D) ** 2

    # Average the power across time to get the power spectral density
    psd = np.mean(S_power, axis=1)

    # Normalize the PSD
    psd_normalized = psd / np.sum(psd)

    # Plot the power spectral density
    frequencies = librosa.fft_frequencies(sr=sr)

    plt.figure(figsize=(12, 6))
    plt.plot(frequencies, psd_normalized)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Normalized Power')
    plt.title('Power Spectral Density')
    #plt.show()

    # Statistical analysis of the PSD to detect white noise
    psd_mean = np.mean(psd_normalized)
    psd_variance = np.var(psd_normalized)
    psd_skewness = skew(psd_normalized)
    psd_kurtosis = kurtosis(psd_normalized)

    print(f"Mean of PSD: {psd_mean}")
    print(f"Variance of PSD: {psd_variance}")
    print(f"Skewness of PSD: {psd_skewness}")
    print(f"Kurtosis of PSD: {psd_kurtosis}")

    # Check if the PSD is flat (indicative of white noise)
    if psd_skewness > 0.1 and psd_kurtosis > 3.1:
        print("The audio signal likely contains white noise.")
    else:
        print("The audio signal does not appear to contain white noise.")

def readAudio(_wav_file):
    with wave.open(_wav_file, 'r') as file:
        meta = file.getparams()
        arr = np.frombuffer(file.readframes(meta.nframes), np.int16)
        arr.shape = -1, 1
        arr = arr.astype(np.float32, order='C')
        return (arr, meta.framerate)
        print(arr.shape)
        print(arr)

def readFile(_pkl_file):
    with open(_pkl_file, 'rb') as f:
        buffer = pickle.load(f)
        return np.array(buffer, dtype=np.float32)
    
def filter_bank(spectrum_db):
    spectrum_db.shape = -1, 1023
    print(spectrum_db.shape)

    spectrum_db_norm = (spectrum_db - np.min(spectrum_db)) / (np.max(spectrum_db) - np.min(spectrum_db))
    num_filters = 200
    num_frequency_bins = spectrum_db_norm.shape[1]
    num_samples = spectrum_db_norm.shape[0]
    max_frequency = 100000  # 100 kHz

    # Define the frequency range based on the number of frequency bins
    frequencies = np.linspace(0.000001, max_frequency - 1, num_frequency_bins)  # Normalized frequency range (0 to 0.5, Nyquist)

    # Create filter bank
    filter_bank = []
    for i in range(num_filters):
        low_freq = frequencies[int(i * num_frequency_bins / num_filters)]
        high_freq = frequencies[int((i + 1) * num_frequency_bins / num_filters) - 1]
        #center_frequencies.append(())
        print('Indexes: ', i, ' -> ', low_freq, ' ', high_freq)
        band = [low_freq / (max_frequency), high_freq / (max_frequency)]
        #print(band)
        sos = signal.butter(2, band, btype='band', output='sos')
        filter_bank.append(sos)

    # Initialize a matrix to store the band energies
    band_energies = np.zeros((num_samples, num_filters))

    # Apply each filter to each sample and measure energy
    for t in range(num_samples):
        for f_idx, sos in enumerate(filter_bank):
            #print(spectrum_db[t, :])
            filtered_signal = signal.sosfilt(sos, spectrum_db_norm[t, :])
            #print(np.max(filtered_signal))
            band_energy = np.sum(filtered_signal ** 2)
            band_energies[t, f_idx] = band_energy

    # Normalize the energies
    #band_energies /= np.max(band_energies)

    # Plot the average energy distribution across the filter bands
    average_band_energies = np.mean(band_energies, axis=0)
    #print(average_band_energies)
    #center_frequencies = [(frequencies[int(i * num_frequency_bins / num_filters)] + frequencies[int((i + 1) * num_frequency_bins / num_filters) - 1]) / 2 for i in range(num_filters)]
    #print(center_frequencies)

    plt.figure(figsize=(10, 6))
    obs_range = (37, 199) #range of interest
    #plt.bar(range(num_filters), average_band_energies)
    center_frequencies = [(frequencies[int(i * num_frequency_bins / num_filters)] + frequencies[int((i + 1) * num_frequency_bins / num_filters) - 1]) / 2 for i in range(num_filters)]
    plt.bar(center_frequencies[obs_range[0]:obs_range[1]], average_band_energies[obs_range[0]:obs_range[1]], width=(max_frequency / num_filters))
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Normalized Energy')
    plt.title('Average Filter Bank Energy Distribution')
    plt.show()

    # Check uniformity of the energy distribution
    mean_energy = np.mean(average_band_energies[obs_range[0]:obs_range[1]])
    std_energy = np.std(average_band_energies[obs_range[0]:obs_range[1]])

    print(f"Mean energy: {mean_energy:.2f}")
    print(f"Standard deviation of energy: {std_energy:.2f}")

    if std_energy < 0.1:  # Adjust threshold based on requirements
        print("The spectrum likely contains white noise.")
    else:
        print("The spectrum does not appear to contain white noise.")

def filter_bank2(spectrum_db:np.array):
    spectrum_db.shape = -1, 1023
    spectrum_db = spectrum_db.transpose()
    print(spectrum_db.shape)
    
    num_filters = 10
    frequency_bins = spectrum_db.shape[0] #1023
    num_samples = spectrum_db.shape[1] #902

    min_db = np.min(spectrum_db)
    max_db = np.max(spectrum_db)
    spectrum_db_norm = (spectrum_db - min_db)/(max_db -min_db)

    # Define the filter bank parameters
    frequencies = np.linspace(0, frequency_bins, num_filters + 1, dtype=int) # Divide frequency range into bands
    print(frequencies.shape)
    # Calculate energy in each filter band
    band_energies = []
    for i in range(num_filters):
        band = spectrum_db_norm[frequencies[i]:frequencies[i+1], :]
        print(band)
        band_energy = np.mean(band)
        band_energies.append(band_energy)

    # Normalize the energies
    band_energies = np.array(band_energies)
    band_energies /= np.max(band_energies)

    # Plot the energies
    plt.figure(figsize=(10, 6))
    plt.bar(range(num_filters), band_energies)
    plt.xlabel('Filter Band')
    plt.ylabel('Normalized Energy')
    plt.title('Filter Bank Energy Distribution')
    plt.show()

    # Check uniformity of the energy distribution
    mean_energy = np.mean(band_energies)
    std_energy = np.std(band_energies)

    print(f"Mean energy: {mean_energy:.2f}")
    print(f"Standard deviation of energy: {std_energy:.2f}")

    if std_energy < 0.1:  # Adjust threshold based on requirements
        print("The audio signal likely contains white noise.")
    else:
        print("The audio signal does not appear to contain white noise.")

def loudness_detector(fft_db):
    fft_db.shape = -1, 1023
    print(fft_db.shape)
    samples = fft_db.shape[0]
    x = np.linspace(0, 30, samples)
    y = []
    for i in range(samples):
        #sum = np.sum(fft_db[i, :])
        mean = np.mean(fft_db[i, :])
        #print('Sum: ', sum, ' Mean: ',  mean)
        if(mean < 0):
            mean = 0.0
        y.append(mean)

    y = np.array(y)
    plt.figure(figsize=(10, 6))
    plt.bar(x, y)

def detector(fft_db:np.array, min_freq=0, max_freq=100000):
    fft_db.shape = -1, 1023
    print(fft_db.shape)
    num_samples = fft_db.shape[0]
    freq = np.fft.rfftfreq(n=2048, d=1/200000)[2:]
    energy = np.zeros((1023, ), dtype=np.float32)
    it = np.nditer(freq, flags=['f_index'])
    for s in range(num_samples):#for each time sample
        bin = fft_db[s, : ] # (1023, )
        for a in it: #iterate over the content of bins
            if((a >= min_freq) and (a <= max_freq)): #filter by freq range
                energy[it.index] = dbsum([energy[it.index], bin[it.index]])

    #print(energy)
    # Check uniformity of the energy distribution
    mean_energy = np.mean(energy)
    std_energy = np.std(energy)

    print(f"Mean energy: {mean_energy:.2f}")
    print(f"Standard deviation of energy: {std_energy:.2f}")

    plt.figure(figsize=(10, 6))
    plt.bar(freq, energy, width=len(freq))
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Energy')
    plt.title('Energy Distribution')


def analyzeSpec(y:np.array, sr:int=100000):
    y.shape = -1, 1023
    print(y.shape)

    fig, ax = plt.subplots()
    ax.set_title('Spectrum Plot - Linear')
    freq = np.fft.rfftfreq(n=2048, d=1/200000)[2:]
    img = ax.imshow(y.transpose(), origin='lower', aspect='auto', cmap='inferno', extent=[0, 60, freq[0], freq[-1]])
    #y = np.squeeze(y)
    #ax.set_xlim((0, 49000))
    #ax.set_xticks(np.arange(0, 5, 0.1))
    print(freq.shape)
    #ax.set_yscale(freq)
    img.set_array(y.transpose())
    fig.colorbar(img, ax=ax, format="%+2.f dB")
    return y

if __name__ == '__main__':

    #graph_spectogram(wav_file)
    #graph_wavfileread(wav_file)
    #sound_pressure(wav_file)
    # air_leak_detection(wav_file)
    # air_leak_detection_autocorr(wav_file)
    # white_noise_detection(wav_file)
    analyzeSpec(readFile(pkl_file))
    detector(readFile(pkl_file))
    plt.show()
