import sys
import numpy as np
import sounddevice as sd
from threading import Thread

class Loop:
    def __init__(self):
        self.audio = np.ones((samplerate * max_loop_duration_s, 2))
        self.widx = 0
        self.ridx = 0
        self.is_recording = False
        self.synced_widx = 0

    def set_is_recording(self):
        self.is_recording = ~self.is_recording
        if self.is_recording:
            print(f'recording loop {loop_idx}...')
        else:
            print(f'...loop {loop_idx} stopped')
            self.increment_loop_idx()

    def write(self, indata, n_frames):
        self.audio[self.widx : self.widx + n_frames] *= indata # TODO apply LPF here instead of output signal
        if self.ridx == 0: # start of loop
            self.audio[:len(ramp_up)] *= ramp_up
        self.widx += n_frames
        if self.widx  > len(self.audio):
            raise Exception(f'maximum loop length of {max_loop_duration_s}s exceeded')
        if is_synced:
            if loop_idx == 0 :
                global base_loop_length
                base_loop_length = self.widx
                self.synced_widx = self.widx
            else:
                self.synced_widx = base_loop_length * int(np.round(self.widx / base_loop_length))
    
    def read(self, n_frames):
        ret = self.audio[self.ridx : self.ridx+n_frames]
        self.ridx += n_frames
        self.ridx = self.ridx % (self.synced_widx if is_synced else self.widx)
        if self.ridx == 0: # end of loop
            ret[-len(ramp_up):] *= ramp_up[::-1]
        return ret
    
    def increment_loop_idx(self):
        global loop_idx
        if loop_idx == n_loop_tracks - 1:
            print(f'using all {n_loop_tracks} available tracks, overwriting...')
            loop_idx = 0 
            loops[loop_idx] = Loop()
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
    for i in [i for i in range(n_loop_tracks) if loops[i].widx > 0 and not loops[i].is_recording]:
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
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            loops[loop_idx].set_is_recording()

def play_audio():
    with stream:
        while True:
            if stream.stopped:
                break

max_loop_duration_s = 30
n_loop_tracks = 8
output_gain = 100
f_c = 1_000
is_synced = True
fade_ms = 500

samplerate = get_samplerate()
omega_c = 2 * np.pi * f_c / samplerate 
alpha = omega_c / (omega_c + 1)
ramp_up = np.arange(samplerate * fade_ms * 1e-3)/(samplerate * fade_ms * 1e-3)
ramp_up = np.stack((ramp_up, ramp_up)).T # stereo

loops = [Loop() for _ in range(n_loop_tracks)]
loop_idx = 0

def callback(indata, outdata, frames, time, status):
    if status:
        print(status)
    tmpdata = output_gain * (indata + read_loops(indata.shape, frames))
    outdata = one_pole_low_pass_filter(tmpdata, outdata)
    if loops[loop_idx].is_recording:
        loops[loop_idx].write(indata, frames)

#sys.exit()
stream = sd.Stream(samplerate=samplerate, callback=callback, latency='low', blocksize=0)
if __name__ == "__main__":
    Thread(target=play_audio).start()
    Thread(target=run_ui).start()

def finite_signal_iir_lpf(input, f_c = 1_000):
    omega_c = 2 * np.pi * f_c / samplerate 
    alpha = omega_c / (omega_c + 1)
    ret = np.zeros(input.shape)
    ret[0] = alpha * input[0]
    for i in range(1, len(input)):
        ret[i] = ret[i-1] + alpha * (input[i] - ret[i-1])
    return ret

def filter_audiofile():
    import soundfile as sf
    x, fs = sf.read('guitar.wav')
    sf.write('lowpass_filtered_guitar.wav', finite_signal_iir_lpf(x, 20), fs)

def plot_frequency_response():
    x = np.random.random((samplerate, 2)) - 0.5
    y = finite_signal_iir_lpf(x, f_c = 20_000)
    from matplotlib import pyplot as plt
    plt.plot(samplerate*np.arange(len(x))/len(x), 20*np.log10(abs(np.fft.fft(x[:, 0]))))
    plt.plot(samplerate*np.arange(len(x))/len(x), 20*np.log10(abs(np.fft.fft(y[:, 0]))))
    plt.xlim(0, samplerate/2)