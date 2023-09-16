import sys
import numpy as np
import soundfile as sf
import sounddevice as sd
from threading import Thread
from time import perf_counter
from datetime import datetime

class Loop:
    def __init__(self):
        self.audio = np.zeros((samplerate * max_loop_duration_s, n_channels))
        self.write_idx = 0
        self.read_idx = 0
        self.is_recording = False
        self.synced_write_idx = 0

    def set_is_recording(self):
        self.is_recording = ~self.is_recording
        if self.is_recording:
            print(f'recording loop {loop_idx}...')
        else:
            print(f'...loop {loop_idx} stopped')
            self.apply_fade_in_out()
            self.increment_loop_idx()
    
    def lowpass_filtered_write(self, indata, n_frames):
        for i in range(n_frames):
            self.audio[self.write_idx + i] = self.audio[self.write_idx + i - 1] + alpha * (indata[i] - self.audio[self.write_idx + i - 1])

    def write(self, indata, n_frames):
        self.audio[self.write_idx : self.write_idx + n_frames] = indata # self.lowpass_filtered_write(indata, n_frames) # 
        self.write_idx += n_frames
        if self.write_idx  > len(self.audio):
            raise Exception(f'maximum loop length of {max_loop_duration_s}s exceeded')
        if is_synced:
            if loop_idx == 0 :
                global base_loop_length
                base_loop_length = self.write_idx
                self.synced_write_idx = self.write_idx
            else:
                synced_write_idx = base_loop_length * int(np.round(self.write_idx / base_loop_length))
                self.synced_write_idx = base_loop_length if synced_write_idx == 0 else synced_write_idx
    
    def read(self, n_frames):
        ret = self.audio[self.read_idx : self.read_idx+n_frames]
        self.read_idx += n_frames
        self.read_idx = self.read_idx % (self.synced_write_idx if is_synced else self.write_idx)
        return ret
    
    def apply_fade_in_out(self):
        n_loop_samples = self.write_idx
        if is_synced and self.synced_write_idx < self.write_idx:
            n_loop_samples = self.synced_write_idx # only fade at this position if shortening the recorded loop for syncing
        if n_loop_samples < len(ramp):
            print(f'{fade_ms}ms fade-in/out longer than loop, reducing fade duration...')
            tmp_ramp = np.stack((np.arange(n_loop_samples)/(n_loop_samples), np.arange(n_loop_samples)/(n_loop_samples))).T # stereo
            self.audio[:len(tmp_ramp)] *= tmp_ramp[:, :n_channels]
            self.audio[:n_loop_samples][-len(tmp_ramp):] *= tmp_ramp[:, :n_channels][::-1]
        else:
            self.audio[:len(ramp)] *= ramp
            self.audio[:n_loop_samples][-len(ramp):] *= ramp[::-1]

    def increment_loop_idx(self):
        global loop_idx
        if loop_idx == n_loop_tracks - 1:
            print(f'all {n_loop_tracks} available tracks in use, overwriting previous loops...')
            loop_idx = 0 
            loops[loop_idx] = Loop() # TODO don't erase previous loop until recording actually starts
        else:
            loop_idx += 1

def get_samplerate():
    output_device = sd.query_devices(kind='output')
    try:
        return int(output_device['default_samplerate'])
    except TypeError:
        sys.exit('no output device available')
    except KeyError:
        sys.exit('no default samplerate available')

def read_loops(indata_shape, n_frames):
    ret = np.zeros(indata_shape)
    for i in [i for i in range(n_loop_tracks) if loops[i].write_idx > 0 and not loops[i].is_recording]:
        ret += loops[i].read(n_frames)
    return ret 

outdata1 = np.zeros((2))
def one_pole_low_pass_filter(indata, outdata):
    global outdata1
    outdata[0] = outdata1 + alpha * (indata[0] - outdata1)
    for i in range(1, len(indata)):
        outdata[i] = outdata[i-1] + alpha * (indata[i] - outdata[i-1])
    outdata1 = outdata[-1]
    return outdata

def run_ui():
    global is_recording_output
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            is_recording_output = ~is_recording_output
            print('storing audio...' if is_recording_output else '...writing stored audio to file')

def play_audio():
    with stream: 
        while True:
            if stream.stopped:
                break

max_loop_duration_s = 30
n_loop_tracks = 8
is_synced = True # False # 
output_gain = 1
fade_ms = 25
f_c = 1_000
n_channels = 1
is_recording_output = False

device_idx = 2
samplerate = int(sd.query_devices(device_idx)['default_samplerate']) # get_samplerate()
omega_c = 2 * np.pi * f_c / samplerate 
alpha = omega_c/ (omega_c + 1)
ramp = np.arange(samplerate * fade_ms * 1e-3)/(samplerate * fade_ms * 1e-3)
ramp = (np.stack((ramp, ramp)).T)[:, :n_channels] # stereo

loop_idx = 0
loops = [Loop() for _ in range(n_loop_tracks)]

output = []
def callback(indata, outdata, frames, time, status):
    global output
    if status:
        print(status)
    outdata[:] = output_gain * (indata + read_loops(indata.shape, frames)) # tmpdata = output_gain * (indata + read_loops(indata.shape, frames))
    #outdata = one_pole_low_pass_filter(tmpdata, outdata) # provides better de-clicking... but why isn't fade-in/out sufficient? syncing caused fades to start at wrong position
    if is_recording_output:
        output.append(outdata.copy())
    elif len(output) > 0:
        sf.write(f'loop_jam_{datetime.now().strftime("%Y%m%d_%H%M%S")}.wav', np.vstack(output), samplerate)
        output = []

    if loops[loop_idx].is_recording:
        loops[loop_idx].write(indata, frames)

#sys.exit()
stream = sd.Stream(device=device_idx, channels=n_channels, samplerate=samplerate, callback=callback, latency='low') #, blocksize=0)
if __name__ == "__main__":
    Thread(target=play_audio).start()
    Thread(target=run_ui).start()


def finite_signal_iir_lpf(input, x1):
    ret = np.zeros(input.shape)
    ret[0] = alpha * input[0]
    for i in range(1, len(input)):
        ret[i] = ret[i-1] + alpha * (input[i] - ret[i-1])
    return ret

def filter_audiofile():
    x, fs = sf.read('guitar.wav')
    sf.write('lowpass_filtered_guitar.wav', finite_signal_iir_lpf(x, 20), fs)

def plot_frequency_response():
    x = np.random.random((samplerate, 2)) - 0.5
    y = finite_signal_iir_lpf(x, f_c = 20_000)
    from matplotlib import pyplot as plt
    plt.plot(samplerate*np.arange(len(x))/len(x), 20*np.log10(abs(np.fft.fft(x[:, 0]))))
    plt.plot(samplerate*np.arange(len(x))/len(x), 20*np.log10(abs(np.fft.fft(y[:, 0]))))
    plt.xlim(0, samplerate/2)