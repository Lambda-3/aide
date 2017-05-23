from sound_play.libsoundplay import SoundClient


class say:
    sound_handle = None

    def __init__(self, text):

        try:
            say.sound_handle.say(text)
        except AttributeError:
            say.sound_handle = SoundClient()
            say.sound_handle.say(text)


class PhoneService:
    def call(self, number, text):
        print(20 * "=")
        print("Calling {}...".format(number))
        print("...connection established, saying text:")
        for row in text.splitlines():
            print("\t" + row)
        print("Hanging up.")
        print(20 * "=")

    def send_message(self, number, message):
        print(20 * "=")
        print("Sending message to {} with content \"{}\".".format(number, message))
        print(20 * "=")


class DummyMover:
    def move(self, coordinates):
        print(20 * "=")
        print("Moving to coordinates {}.".format(coordinates))

        print(20 * "=")

    def get_nearest_corner(self):
        return (1.3, 3.7)


phone_service = PhoneService()
move_service = DummyMover()

# def reset_face_tracking():
#     FaceServiceHandler.call_reset_service()
