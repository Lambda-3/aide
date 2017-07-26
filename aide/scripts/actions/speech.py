from sound_play.libsoundplay import SoundClient


def say(text):
    _say(str(text))
    # os.system('espeak "{}" 2> /dev/null'.format(text))


def pprint(text):
    print(text)


class _say(object):
    sound_handle = SoundClient()

    def __init__(self, text):
        try:
            self.sound_handle.say(text)
        except AttributeError:
            self.sound_handle.say(text)
