import requests
import subprocess
import json
from sound_play.libsoundplay import SoundClient


class say:
    sound_handle = None

    def __init__(self, text):

        try:
            say.sound_handle.say(text)
        except AttributeError:
            say.sound_handle = SoundClient()
            say.sound_handle.say(text)


class LocationService:
    API_KEY = "AIzaSyA_oCs9GGnLGXyMt7SQHqDv3ncHCmYEBBc"
    ENDPOINT = "https://www.googleapis.com/geolocation/v1/geolocate?key={}".format(API_KEY)

    def get_location(self):
        mac_adresses = subprocess.check_output(
            ["nmcli", "-t", "-m", "multiline", "-f", "BSSID,CHAN,SIGNAL", "dev", "wifi", "list"])
        format_rows = lambda x: x.split(":", 1)[1] if not x.startswith("SIGNAL") else (int(x.split(":", 1)[1]) / 2) - 92

        wifi_stations = [dict(zip(["macAddress", "channel", "signalStrength"], map(format_rows, row.split())))
                         for row in mac_adresses.split("BSSID") if row]

        data = {'considerIp': 'false',
                'wifiAccessPoints': wifi_stations}
        headers = {
            'content-type': "application/json"
        }

        response = requests.request("POST", LocationService.ENDPOINT, data=json.dumps(data), headers=headers)

        response.raise_for_status()

        response = response.json()
        return response["location"]


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
location_service = LocationService()

# def reset_face_tracking():
#     FaceServiceHandler.call_reset_service()
