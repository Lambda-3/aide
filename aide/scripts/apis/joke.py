import requests

ENDPOINT = "https://api.icndb.com/jokes/random/?firstName=Mario&lastName=The%20Robot&limitTo=[nerdy]"


def get_random_joke():
    headers = {
        'content-type': "application/json"
    }

    response = requests.request("GET", ENDPOINT, headers=headers)

    response.raise_for_status()

    response = response.json()
    return response["value"]["joke"]
