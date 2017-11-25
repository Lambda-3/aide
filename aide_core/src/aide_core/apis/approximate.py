import inspect

from aide_messages.msg import Event
from aide_core.apis import storage
from enum import Enum
from rospy import loginfo

from aide_core.apis import ros_services
from aide_core.apis.util import camel_case_to_underscore, underscore_to_camel_case


def create(**params):
    try:
        params['collection']
    except KeyError:
        # noone needs to understands this. just gets the collection name from Enums defined name
        name = inspect.stack()[1][4][0].split("=", 1)[0].strip()
        params['collection'] = camel_case_to_underscore(name)

    if "select_only" not in params:
        params['select_only'] = ["name"]

    try:
        params['select_only'][0]
    except IndexError:
        params['select_only'] = [params['select_only']]

    try:
        params['word_format']
    except KeyError:
        params['word_format'] = "{name}"

    return params


class Types(Enum):
    Apis = create()

    ApiFuncs = create(select_only=["api", "doc", "name", "args", "hinted_args"], word_format="{api} {name}")

    Actions = create()

    ActionFuncs = create(select_only=["api", "doc", "name", "args", "hinted_args"], word_format="{api} {name}")

    Events = create(select_only=ros_services.get_slots(Event))

    Routines = create()
    
    Something = create()


def get_semantically_related(to, type, top_k=3):
    """
    
    :param to:
    :type to: str
    :param type:
    :type type: str
    :param top_k:
    :type  top_k: int
    :return: 
    """
    type_name = underscore_to_camel_case(type)
    loginfo(type_name)
    try:
        parsed_type = Types[type_name].value
    except KeyError:
        return None

    return storage.find_related(related_to=to, top_k=top_k, **parsed_type)
