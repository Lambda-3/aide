from sound_play.libsoundplay import SoundClient


def say(text):
    _say(text)


class _say(object):
    sound_handle = SoundClient()

    def __init__(self, text):
        try:
            self.sound_handle.say(text, voice='voice_en1_mbrola')
        except AttributeError:
            self.sound_handle.say(text)
