from sound_play.libsoundplay import SoundClient
from face_rec import ServiceHandler as FaceServiceHandler

VOICE = "voice_en1_mbrola"
sound_handle = SoundClient(blocking=False)


class say:
    sound_handle = None

    def __init__(self, text):

        try:
            say.sound_handle.say(text)
        except AttributeError:
            say.sound_handle = SoundClient()
            say.sound_handle.say(text)


def reset_face_tracking():
    FaceServiceHandler.call_reset_service()
