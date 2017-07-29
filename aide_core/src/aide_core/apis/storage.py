import inspect
from functools import partial

import genpy
import pymongo
from bson import ObjectId

from aide_core.apis import indra
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as rtd

__client = pymongo.MongoClient()


class CollectionWrapper(object):
    def __init__(self, collection):
        self.collection = collection

    delete_many = lambda self, *args, **kwargs: partial(delete_many, collection=self.collection)(*args, **kwargs)

    insert = lambda self, *args, **kwargs: partial(insert, collection=self.collection)(*args, **kwargs)

    insert_many = lambda self, *args, **kwargs: partial(insert_many, collection=self.collection)(*args, **kwargs)

    find = lambda self, *args, **kwargs: partial(find, collection=self.collection)(*args, **kwargs)

    find_one = lambda self, *args, **kwargs: partial(find_one, collection=self.collection)(*args, **kwargs)

    replace_one = lambda self, *args, **kwargs: partial(replace_one, collection=self.collection)(*args, **kwargs)

    find_related = lambda self, *args, **kwargs: partial(find_related, collection=self.collection)(*args, **kwargs)


def get_collection(name=None, indices=list()):
    name = _cure_args(name)
    collection = __client.db[name]
    for index in indices:
        collection.create_index(index, unique=True)
    return CollectionWrapper(collection)


def delete_many(where, collection=None, *args, **kwargs):
    collection = _cure_args(collection)
    __client.db[collection].delete_many(filter=where, *args, **kwargs)


def insert_many(entries, collection=None, *args, **kwargs):
    entries = [rtd(entry) if isinstance(entry, genpy.Message) else entry for entry in entries]
    collection = _cure_args(collection)
    __client.db[collection].insert_many(entries, *args, **kwargs)


def find(where={}, select_only=None, include_id=False, collection=None, *args, **kwargs):
    collection = _cure_args(collection)
    if select_only:
        if isinstance(select_only, list):
            select_only = {k: True for k in select_only}
        select_only.update({"_id": include_id})
    elif not include_id:
        select_only = {"_id": False}

    return __client.db[collection].find(filter=where, projection=select_only, *args, **kwargs)


def replace_one(replace_with, where=dict(), collection=None, *args, **kwargs):
    replace_with = rtd(replace_with) if isinstance(replace_with, genpy.Message) else replace_with
    collection = _cure_args(collection)
    return __client.db[collection].replace_one(filter=where, replacement=replace_with)


def find_one(where, select_only=None, include_id=False, collection=None, *args, **kwargs):
    collection = _cure_args(collection)
    if select_only:
        if isinstance(select_only, list):
            select_only = {k: True for k in select_only}
        select_only.update({"_id": include_id})
    return __client.db[collection].find_one(filter=where, projection=select_only, *args, **kwargs)


def find_related(related_to, select_only=None, word_format="{name}", include_id=False, top_k=5,
                 collection=None, *args, **kwargs):
    collection = _cure_args(collection)

    result = indra.sort_by_relatedness_with_id(related_to,
                                               find(collection=collection, select_only=select_only,
                                                    include_id=True),
                                               format=word_format)
    index = top_k if len(result) > top_k else len(result)
    if select_only:
        if isinstance(select_only, list):
            select_only = {k: True for k in select_only}
        select_only.update({"_id": include_id})
    elif not include_id:
        select_only = {"_id": False}
    return [find_one(collection=collection, where={"_id": ObjectId(rsp)}, select_only=select_only,
                     include_id=include_id) for rsp in result][:index]


def insert(entry, collection=None, *args, **kwargs):
    collection = _cure_args(collection)
    entry = rtd(entry) if isinstance(entry, genpy.Message) else entry
    __client.db[collection].insert(entry, *args, **kwargs)


def _cure_args(collection):
    if not collection:
        collection = inspect.stack()[1][1].rsplit("/")[-1]
    try:
        collection = collection.name
    except AttributeError:
        pass
    return collection
