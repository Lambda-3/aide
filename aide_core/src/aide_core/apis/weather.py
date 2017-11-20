import requests

from aide_core import credentials
import location

API_KEY = credentials.OPEN_WEATHER_MAP_API_KEY
ENDPOINT = "http://api.openweathermap.org/data/2.5/weather?&units=metric&appid={}".format(API_KEY)


def get_weather(latitude, longitude):
    """

    :param latitude:
    :type: float
    :param longitude:
    :type: float
    :return:
    """
    headers = {
        'content-type': "application/json"
    }
    link = ENDPOINT+ "&lat={}&lon={}".format(latitude, longitude)
    response = requests.request("GET", link, headers=headers)

    response.raise_for_status()

    response = response.json()
    print type(response['weather'])
    return {
        "weather_type": response['weather'][0]['description'],
        "temperature": response['main']['temp'],
        "city": response['name'],
        "country": response['sys']['country']
    }


def get_weather_at_my_place_human_readable():
    return get_weather_at_my_place_human_readable(**location.get_location())
    

def get_weather_human_readable(lat, lng):
    weather= get_weather(lat, lng)
    return """The weather at {city}, {country} is {weather_type}, it has {temperature} degree(s).""".format(**weather)
