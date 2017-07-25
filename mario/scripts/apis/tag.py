import atexit
import json

from rospy import loginfo

import config
_places = dict()


def tag_place(x, y, name):
    loginfo("Tagging coordinates ({},{}) with name {}".format(x, y, name))
    _places[name] = (x, y)


def get_tagged_coordinates(name):
    try:
        return _places[name]
    except KeyError:
        return None


def _save_places():
    places = json.dumps(_places)
    loginfo("saving {}".format(places))
    with open(config.PROJECT_PATH + "/.places", "w") as f:
        f.write(places)


def _load_places():
    loginfo("loading saved places...")
    with open(config.PROJECT_PATH + "/.places", "r") as f:
        places = json.loads(f.read())
        loginfo("Loaded {}".format(places))
        _places.update(places)


atexit.register(_save_places)
if not _places:
    _load_places()