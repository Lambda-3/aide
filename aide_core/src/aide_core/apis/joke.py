import requests

ENDPOINT = "https://api.icndb.com/jokes/random/?firstName=My&lastName=Robot&limitTo=[nerdy]"


def get_random_joke():
    """
    Gets a random joke from the api specified by ENDPOINT.
    :return: A random joke.
    :rtype: str
    """
    headers = {
        'content-type': "application/json"
    }


    response = requests.request("GET", ENDPOINT, headers=headers)

    response.raise_for_status()

    response = response.json()
    return response["value"]["joke"]
