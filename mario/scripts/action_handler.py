#!/usr/bin/env python
import reimport as reimport
import roslib
import rospy
from std_msgs.msg import Bool

import apis
import actions
from apis import storage, util

roslib.load_manifest("mario")
from rospy.core import logerror

from rospy import loginfo

import inspect
import json

import os
import traceback

from mario_messages.srv import (CallFunction, AddActionProvider, GetAllActionProviders, GetActionProvider)

import config

from apis.ros_services import get_service_handler

apis.__update()
actions.__update()


class ActionHandler:
    def __init__(self):
        self.function_storage = storage.get_collection('action_funcs')
        self.api_storage = storage.get_collection("actions")

        action_provider_files = self.get_all_action_provider_files()

        for file_name in action_provider_files:
            with open(file_name, "r") as f:
                loginfo("Working on: {}".format(file_name))
                file_content = f.read()
            self.add_action_provider(f.name.rsplit("/", 1)[-1], file_content)

    def get_action_provider(self, name):
        try:
            with open("{}/{}.py".format(config.ACTION_PROVIDERS_PATH, name), "r") as f:
                file_content = f.read()
            return file_content
        except IOError as e:
            if "No such file or directory" in e:
                loginfo(e.message)
                return None
            else:
                raise IOError(e)

    def reload_api_references(self):
        for module in reimport.modified():
            if not module == "__main__":
                loginfo("Module {} changed. reloading...".format(module))
                reimport.reimport(module)

    def add_action_provider(self, name, file_content):
        loginfo("Adding action provider %s" % name)
        if name.endswith(".py"):
            name = name[:-3]
        file_name = name + ".py"
        loginfo("File name: ".format(file_name))

        try:
            # todo maybe use lint
            loginfo("compiling...")
            compile(file_content, file_name, "exec")
        except Exception as e:
            return (False, traceback.format_exc(limit=0))

        loginfo("compiled!")
        with open(config.ACTION_PROVIDERS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        try:
            loginfo("   importing...")
            exec """import actions.{0} as {0}\nimp.reload({0})""".format(name)
            imported_action_provider = eval(name)

        except Exception as e:
            logerror(traceback.format_exc())
            return (False, traceback.format_exc(1))
        loginfo("   {} imported!".format(imported_action_provider))

        actions = [
            {"name": f[0],
             "doc" : util.get_doc(f[1]),
             "api" : name,
             "args": inspect.getargspec(f[1]).args}
            for f in
            [
                f for f in inspect.getmembers(imported_action_provider, predicate=inspect.isfunction) if
                f[1].__module__ == imported_action_provider.__name__
            ]
        ]

        api = {"name": name, "doc": util.get_doc(imported_action_provider)}

        loginfo("   adding funcs: {}".format(actions))

        self.function_storage.delete_many({"api": name})
        self.api_storage.delete_many({"name": name})

        self.function_storage.insert_many(actions)
        self.api_storage.insert(api)

        loginfo("   setting: self.{} = {}".format(name, imported_action_provider))
        setattr(self, name, imported_action_provider)
        return (True, "")

    def call_func(self, func_name, **kwargs):
        api, func = func_name.rsplit(".", 1)
        try:
            new_kwargs = dict()
            for k, v in kwargs.items():
                loginfo("Argument: {} = {}".format(k, v))
                if isinstance(v, str) or isinstance(v, unicode) and v.startswith("eval:"):
                    v = str(v[5:])
                    loginfo("Trying to eval: apis." + v)
                    result = eval("apis." + v)
                    new_kwargs[k] = result
                    loginfo("Evaluated {0} as apis.{0}. Result is {1}.".format(v, result))
                else:
                    loginfo("Assuming argument is literal.")
                    new_kwargs[k] = v
            loginfo("Calling function {}.{} with args {}".format(api, func, new_kwargs))
            getattr(getattr(self, api), func)(**new_kwargs)
        except KeyError:
            raise ValueError("There is no function {} in api {}!".format(func, api))

    def get_all_actions_providers(self):
        return [action_provider_path.rsplit("/", 1)[-1].rsplit(".py", 1)[-2] for action_provider_path in
                self.get_all_action_provider_files()]

    def get_all_action_provider_files(self):
        """

        :rtype: generator
        """
        api_files = []
        for (dirpath, _, file_names) in os.walk(config.ACTION_PROVIDERS_PATH):
            api_files.extend([dirpath + "/" + x for x in file_names if x.endswith(".py") and not x.startswith(
                "__")])
        loginfo(api_files)
        return api_files

        # def get_best_matches(self, name, top_k):
        #
        #     """
        #
        #     :rtype: list
        #     """
        #     # build pairs from given name and every func + their api in database. use id to later identify
        #     # pairs = [{"t1": name.replace("_", " "),
        #     #           "t2": "id={id} {api} {name}".format(id=row['_id'], name=row['name'].replace("_", " "),
        #     #                                               api=row['api'].rsplit("_")[0])}
        #     #          for row in self.actions_table.find({}, {"name": True, "api": True, "_id": True})]
        #     #
        #     # data = {'corpus'       : 'wiki-2014',
        #     #         'model'        : 'W2V',
        #     #         'language'     : 'EN',
        #     #         'scoreFunction': 'COSINE', 'pairs': pairs}
        #     #
        #     # headers = {
        #     #     'content-type': "application/json"
        #     # }
        #     #
        #     # response = requests.request("POST", "http://localhost:8916/relatedness", data=json.dumps(data), headers=headers)
        #     #
        #     # response.raise_for_status()
        #     #
        #     # response = response.json()['pairs']
        #     #
        #     # index = top_k if len(response) > top_k else len(response)
        #     #
        #     # return [self.actions_table.find_one({"_id": ObjectId(rsp['t2'].split(" ", 1)[0].split("=")[1])},
        #     #                                     {"_id": False})
        #     #         for rsp in sorted(response, reverse=True)][:index]
        #     return self.function_storage.find_related(name, ["name", "api", "doc", "args"], "{api} {name}", top_k=top_k)


def main():
    rospy.init_node("actions")
    loginfo("Creating action handler...")
    action_handler = ActionHandler()
    loginfo("Registering services...")

    get_service_handler(CallFunction).register_service(
        lambda func_name, kwargs: action_handler.call_func(func_name, **json.loads(kwargs))
    )
    rospy.Subscriber("/mario/update_apis", Bool, lambda x: action_handler.reload_api_references() if x.data else ())

    get_service_handler(GetActionProvider).register_service(lambda name: action_handler.get_action_provider(name) or ())
    get_service_handler(GetAllActionProviders).register_service(action_handler.get_all_actions_providers())
    get_service_handler(AddActionProvider).register_service(action_handler.add_action_provider)

    loginfo("Registered services. Spinning.")

    rospy.spin()


if __name__ == "__main__":
    main()
