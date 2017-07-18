from enum import Enum

from storage import find_related


class Types(Enum):
    Apis = {
        "collection" : "apis",
        "select_only": "name"
    }

    ApiFunctions = {}

    Actions = {}
