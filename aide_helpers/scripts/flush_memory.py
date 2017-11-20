#!/usr/bin/env python
import pymongo

client = pymongo.MongoClient()

evts = client.db['event_listeners']
evts.delete_many({})

routines = client.db['routines']
routines.delete_many({})

apis = client.db['apis']
apis.delete_many({})

api_funcs = client.db['api_funcs']
api_funcs.delete_many({})

actions = client.db['actions']
actions.delete_many({})

action_funcs = client.db['action_funcs']
action_funcs.delete_many({})
