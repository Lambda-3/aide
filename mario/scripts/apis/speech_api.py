from sound_play.libsoundplay import SoundClient
from rospy import loginfo, logdebug


class SpeechAPI(object):
    sound_handle = SoundClient()

    def say(self, text):
        self.sound_handle.say(text)
