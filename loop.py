import sys, threading
import numpy as np
import sounddevice as sd

def get_samplerate():
    output_device = sd.query_devices(kind='output')
    try:
        return int(output_device['default_samplerate'])
    except TypeError:
        sys.exit('no output device available')
    except KeyError:
        sys.exit('no default samplerate available')

def callback(indata, outdata, frames, time, status):
    global widx, ridx
    if status:
        print(status)

    if is_recording:
        outdata[:] = indata
        loop[widx:widx + frames] = indata
        widx += frames
        if widx > len(loop):
            raise Exception(f'maximum loop length of {max_loop_duration_s}s exceeded')
    else:
        if widx > 0:
            outdata[:] = indata + loop[ridx%widx:ridx%widx + frames]
            ridx += frames
        else:
            outdata[:] = indata

def run_ui():
    global is_recording
    while True:
        txt = input()
        if txt.casefold() == 'q':
            stream.stop()
            sys.exit(0)
        elif txt == '':
            is_recording = ~is_recording
            print('recording...' if is_recording else '...stopped')  

def play_audio():
    with stream:
        while True:
            if stream.stopped:
                break

max_loop_duration_s = 30
samplerate  = get_samplerate()
stream = sd.Stream(samplerate=samplerate, callback=callback)

loop = np.zeros((samplerate * max_loop_duration_s, 2))
widx, ridx, is_recording = 0, 0, False
if __name__ == "__main__":
    threading.Thread(target=play_audio).start()
    threading.Thread(target=run_ui).start()