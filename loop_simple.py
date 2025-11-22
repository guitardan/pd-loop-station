import sys, threading
import sounddevice as sd
import numpy as np

class AudioBuffer:
    def __init__(self, samplerate, indata_shape, max_duration_sec=60):
        self.widx = 0
        self.ridx = 0
        # data buffer length is multiple of indata shape for easier filling
        self.data = np.zeros((indata_shape[0] * (samplerate * max_duration_sec//indata_shape[0]), indata_shape[1]))

    def write(self, indata):
        if self.widx >= len(self.data):
            print('loop buffer overflow, resetting index to 0 and overwriting data')
            self.widx = 0

        self.data[self.widx:self.widx + len(indata), :] = indata
        self.widx += len(indata)
    
    def read(self, n_frames):
        ret = self.data[self.ridx : self.ridx + n_frames, :]
        self.ridx += n_frames
        if self.ridx >= self.widx:
            self.ridx = 0
        return ret

# # Cross-correlation stuff that was probably for pitch detection, but never fully implemented
# def corr(x, h):
#     ret = 0
#     for i in range(len(x)-len(h)+1):
#         ret += (np.dot(h, x[i:i+len(h)]))**2
#     return ret

# def get_corr_kernels(f0 = 880, n_cycles = 10):
#     f0s = [f0*(2**(1/12))**i for i in range(12)]
#     kernels = []
#     for f in f0s:
#         t_f = n_cycles/f
#         t = np.arange(t_f*samplerate)/samplerate
#         y = np.sin(2 * np.pi * f * t)
#         kernels.append(y)

def get_samplerate():
    output_device = sd.query_devices(kind='output')
    try:
        return output_device['default_samplerate']
    except TypeError:
        print('no output device available, terminating...')
        sys.exit(0)
    except KeyError:
        print('no default samplerate available, terminating...')
        sys.exit(0)

samplerate = int(get_samplerate())
indata_shape = (512, 2)

loops = [AudioBuffer(samplerate, indata_shape, max_duration_sec=60)]

is_recording = False
loop_idx = 0
def callback(indata, outdata, frames, time, status):
    if status:
        print(status)
    
    if is_recording:
        loops[loop_idx].write(indata)
        outdata[:] = indata
    else:
        outdata[:] = indata + loops[loop_idx].read(frames)

stream = sd.Stream(callback=callback)
def run_ui():
    global is_recording, loop_idx
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            is_recording = not is_recording
            if is_recording:
                if loops[loop_idx].widx == 0:
                    print('RECORDING')
                else:
                    loops[loop_idx].widx = 0
                    print('RE-RECORDING')
            else:
                print('STOPPING')
        elif txt == ' ':
            loop_idx += 1
            print(f'LOOP INDEX: {loop_idx}')

def play_audio():
    with stream:
        while True:
            if stream.stopped:
                break

if __name__ == "__main__":
    threading.Thread(target=play_audio).start()
    threading.Thread(target=run_ui).start()
    sys.exit(0)
