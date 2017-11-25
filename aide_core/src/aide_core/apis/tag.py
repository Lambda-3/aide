"""
Example API to tag places on the map.
"""

import atexit
from rospy import loginfo
from aide_core.apis import storage

_places = dict()


def tag_place(x, y, name):
    loginfo("Tagging coordinates ({},{}) with name {}".format(x, y, name))
    _places[name] = (x, y)


def get_tagged_coordinates(name):
    try:
        return tuple(_places[name])
    except KeyError:
        return None


def _save_places():
    places = _places
    loginfo("saving {}".format(places))
    storage.get_collection("places").replace_one(_places)


def _load_places():
    loginfo("loading saved places...")
    loaded_places = storage.get_collection("places").find_one(where={})
    if loaded_places:
        _places.update(loaded_places)


atexit.register(_save_places)
if not _places:
    _load_places()
