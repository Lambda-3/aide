import subprocess
import os


try:
    use_espeak = True
    from sound_play.libsoundplay import SoundClient
    _sound_play = not use_espeak
except ImportError:
    _sound_play = False


def say(text):
    # if sound_play exists
    if _sound_play:
        _say(str(text))
    else:
        # try espeak
        try:
            subprocess.call(['espeak', str(text)], stdout=open(os.devnull, 'w'), stderr=subprocess.STDOUT)
        except OSError:
            # if espeak doesnt exist, just print the text
            print(text)


def pprint(text):
    print(text)


class _say(object):
    sound_handle = SoundClient()

    def __init__(self, text):
        try:
            self.sound_handle.say(text)
        except AttributeError:
            self.sound_handle.say(text)
