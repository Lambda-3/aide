from sound_play.libsoundplay import SoundClient
from face_rec import ServiceHandler as FaceServiceHandler

VOICE = "voice_en1_mbrola"
sound_handle = SoundClient(blocking=False)


def say(text):
    if text:
        sound_handle.say(text, VOICE)


def reset_face_tracking():
    FaceServiceHandler.call_reset_service()
