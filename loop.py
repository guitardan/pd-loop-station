import sys, threading
import numpy as np
import sounddevice as sd

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

def callback(indata, outdata, frames, time, status):
    global loop_idx
    if status:
        print(status)
    if loop_idx < len(loop):
        loop[loop_idx%len(loop):loop_idx%len(loop) + frames] = indata
        outdata[:] = indata
    else:
        outdata[:] = indata + loop[loop_idx%len(loop):loop_idx%len(loop) + frames]
    loop_idx += frames

def run_ui():
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            print('enter')

def play_audio():
    with stream:
        while True:
            if stream.stopped:
                break

n_frames = 512
samplerate  = get_samplerate()
stream = sd.Stream(samplerate=samplerate, callback=callback)

loop = np.zeros((int(samplerate / n_frames) * n_frames, 2))
loop_idx, is_recording = 0, False
if __name__ == "__main__":
    threading.Thread(target=play_audio).start()
    threading.Thread(target=run_ui).start()