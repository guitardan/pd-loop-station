import sys
import numpy as np
import sounddevice as sd
from threading import Thread

class Loop: # TODO enforce audio length multiples of base loop
    def __init__(self):
        self.audio = np.zeros((samplerate * max_loop_duration_s, 2))
        self.widx = 0
        self.ridx = 0
        self.is_recording = False

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
        ret += loops[i].audio[loops[i].ridx : loops[i].ridx+n_frames]
        loops[i].ridx += n_frames
        loops[i].ridx = loops[i].ridx % loops[i].widx
    return ret

def record_loop(indata, n_frames):
    loops[loop_idx].audio[
        loops[loop_idx].widx : loops[loop_idx].widx + n_frames
        ] = indata # TODO apply LPF here instead of output signal
    loops[loop_idx].widx += n_frames
    if loops[loop_idx].widx  > len(loops[loop_idx].audio):
        raise Exception(f'maximum loop length of {max_loop_duration_s}s exceeded')

def set_recording_flag():
    global loop_idx
    loops[loop_idx].is_recording = ~loops[loop_idx].is_recording
    if loops[loop_idx].is_recording:
        print(f'recording loop {loop_idx}...')
    else:
        print(f'...loop {loop_idx} stopped')
        if loop_idx == n_loop_tracks - 1:
            print(f'using all {n_loop_tracks} available loop tracks, overwriting...')
            loop_idx = 0
            loops[loop_idx].ridx = loops[loop_idx].widx = 0
        else:
            loop_idx += 1   

outdata1 = np.zeros((2))
def one_pole_low_pass_filter(indata, outdata):
    global outdata1
    outdata[0] = outdata1 + alpha * (indata[0] - outdata1)
    for i in range(1, len(indata)):
        outdata[i] = outdata[i-1] + alpha * (indata[i] - outdata[i-1])
    outdata1 = outdata[-1]
    return outdata

def run_ui():
    global f_c, alpha
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            set_recording_flag()

def play_audio():
    with stream:
        while True:
            if stream.stopped:
                break

max_loop_duration_s = 30
n_loop_tracks = 8
output_gain = 1
f_c = 1_000

samplerate  = get_samplerate()
omega_c = 2 * np.pi * f_c / samplerate 
alpha = omega_c / (omega_c + 1)

loops = [Loop() for _ in range(n_loop_tracks)]
loop_idx = 0

def callback(indata, outdata, frames, time, status):
    if status:
        print(status)
    indata = output_gain * (indata + read_loops(indata.shape, frames))
    outdata = one_pole_low_pass_filter(indata, outdata)
    if loops[loop_idx].is_recording:
        record_loop(indata, frames)

stream = sd.Stream(samplerate=samplerate, callback=callback)
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