#!/usr/bin/env python
import imp
import inspect
import json
import os
import traceback

import reimport as reimport
import roslib
import rospy
from aide_messages.srv import (CallFunction, AddActionProvider, GetAllActionProviders, GetActionProvider)
from rospy import loginfo
from rospy.core import logerror
from std_msgs.msg import Bool

import actions
import apis.storage
import apis.util
from aide_core import config
from apis.ros_services import get_service_handler

roslib.load_manifest("aide_core")
apis.__update()
actions.__update()


class ActionHandler:
    def __init__(self):
        self.function_storage = apis.storage.get_collection('action_funcs')
        self.api_storage = apis.storage.get_collection("actions")

        action_provider_files = self.get_all_action_provider_files()

        for file_name in action_provider_files:
            with open(file_name, "r") as f:
                loginfo("Working on: {}".format(file_name))
                file_content = f.read()
            self.add_action_provider(f.name.rsplit("/", 1)[-1], file_content, loading=True)

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
        for module in reimport.modified(config.APIS_PATH):
            if not module == "__main__":
                loginfo("Module {} changed. reloading...".format(module))
                reimport.reimport(module)

    def add_action_provider(self, name, file_content, loading=False):
        loginfo("Adding action provider %s" % name)
        if name.endswith(".py"):
            name = name[:-3]
        file_name = name + ".py"
        loginfo("File name: ".format(file_name))

        loginfo("compiling...")
        try:
            compile(file_content, file_name, "exec")
        except Exception:
            return (False, traceback.format_exc(limit=0))

        loginfo("compiled!")
        with open(config.ACTION_PROVIDERS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        lint_output = ""
        if not loading:
            loginfo("   applying Pylint...")
            lint_output = apis.util.apply_lint(config.ACTION_PROVIDERS_PATH + "/" + file_name)

        try:
            loginfo("   importing...")
            exec ("""import actions.{0} as {0}""".format(name))
            imported_action_provider = eval(name)
            imp.reload(imported_action_provider)

        except Exception:
            logerror(traceback.format_exc())
            return (False, lint_output + traceback.format_exc(1))
        loginfo("   {} imported!".format(imported_action_provider))
        actions = [
            {
                "name": f[0],
                "doc": apis.util.get_doc(f[1]),
                "api": name,
                "args": inspect.getargspec(f[1]).args,
                "hinted_args": apis.util.get_type_hints(f[1])[0]
            }
            for f in
            [
                f for f in inspect.getmembers(imported_action_provider, predicate=inspect.isfunction) if
                f[1].__module__ == imported_action_provider.__name__ and not f[0].startswith("_")
            ]
        ]

        api = {"name": name, "doc": apis.util.get_doc(imported_action_provider)}

        loginfo("   adding funcs: {}".format(actions))

        self.function_storage.delete_many({"api": name})
        self.api_storage.delete_many({"name": name})

        self.function_storage.insert_many(actions)
        self.api_storage.insert(api)

        loginfo("   setting: self.{} = {}".format(name, imported_action_provider))
        setattr(self, name, imported_action_provider)
        return True, lint_output

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


def main():
    rospy.init_node("actions")
    loginfo("Creating action handler...")
    action_handler = ActionHandler()
    loginfo("Registering services...")

    get_service_handler(CallFunction).register_service(
        lambda func_name, kwargs: action_handler.call_func(func_name, **json.loads(kwargs))
    )
    rospy.Subscriber("/aide/update_apis", Bool, lambda x: action_handler.reload_api_references() if x.data else ())

    get_service_handler(GetActionProvider).register_service(lambda name: action_handler.get_action_provider(name) or ())
    get_service_handler(GetAllActionProviders).register_service(action_handler.get_all_actions_providers)
    get_service_handler(AddActionProvider).register_service(action_handler.add_action_provider)

    loginfo("Registered services. Spinning.")

    rospy.spin()


if __name__ == "__main__":
    main()
