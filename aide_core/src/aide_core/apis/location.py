import subprocess

import requests
import json

from aide_core import credentials

API_KEY = credentials.GOOGLE_LOCATION_API_KEY
ENDPOINT = "https://www.googleapis.com/geolocation/v1/geolocate?key={}".format(API_KEY)


def get_location():
    # get all wifis using nmcli.
    mac_adresses = subprocess.check_output(
        ["nmcli", "-t", "-m", "multiline", "-f", "BSSID,CHAN,SIGNAL", "dev", "wifi", "list"]
    )

    format_rows = lambda x: x.split(":", 1)[1] if not x.startswith("SIGNAL") else (int(x.split(":", 1)[1]) / 2) - 92

    # apply formatting and zip with field descriptions to comply to the geolocate api format
    wifi_stations = [
        dict(
            zip(["macAddress", "channel", "signalStrength"], map(format_rows, row.split()))
        ) for row in mac_adresses.split("BSSID") if row
    ]

    data = {
        'considerIp': 'true',
        'wifiAccessPoints': wifi_stations
    }


    headers = {
        'content-type': "application/json"
    }

    response = requests.request("POST", ENDPOINT, data=json.dumps(data), headers=headers)

    response.raise_for_status()

    response = response.json()
    return response["location"]


def get_location_human_readable():
    location = get_location()
    return """You are at {lat:.2f} latitude and {lng:.2f} longitude.""".format(**location)