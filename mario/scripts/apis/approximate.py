import inspect

from enum import Enum
from rospy import loginfo
#
from apis import storage
from apis.util import camel_case_to_underscore, underscore_to_camel_case


def create(**params):
    try:
        params['collection']
    except KeyError:
        # noone needs to understands this. just gets the collection name from Enums defined name
        name = inspect.stack()[1][4][0].split("=", 1)[0].strip()
        params['collection'] = camel_case_to_underscore(name)
    try:
        params['select_only']
        try:
            params['select_only'][0]
        except IndexError:
            params['select_only'] = [params['select_only']]
    except KeyError:
        params['select_only'] = ["name"]

    try:
        params['word_format']
    except KeyError:
        params['word_format'] = "{name}"

    return params


class Types(Enum):
    Apis = create()

    ApiFuncs = create(select_only=["api", "name", "args"], word_format="{api} {name}")

    Actions = create()

    ActionFuncs = create(select_only=["api", "name", "args"], word_format="{api} {name}")

    Events = create(select_only=["name", "params"])

    Routines = create()

def get_semantically_related(to, type, top_k=3):
    """
    
    :param name: 
    :type name: str
    :param type:
    :type type: str
    :param top_k:
    :return: 
    """
    type_name = underscore_to_camel_case(type)
    loginfo(type_name)
    try:
        parsed_type = Types[type_name].value
    except KeyError:
        return None

    return storage.find_related(related_to=to, top_k=top_k, **parsed_type)
