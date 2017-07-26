#!/usr/bin/env python

import roslib
import rospy
import imp
import config
import os
import inspect
import traceback

from pylint import epylint as lint

from apis import storage, util
from apis.ros_services import get_service_handler

from rospy import loginfo
from rospy.core import logerror, logwarn

from std_msgs.msg import Bool
from aide_messages.srv import AddApi, GetAllApis, GetApi

roslib.load_manifest("aide")


class ApiHandler:
    def __init__(self, notify_function):
        self.function_storage = storage.get_collection("api_funcs")
        self.api_storage = storage.get_collection("apis")

        api_files = self.get_all_api_files()

        for file_name in api_files:
            loginfo("Working on: {}".format(file_name))
            with open(file_name, "r") as f:
                file_content = f.read()
            self.add_api(f.name.rsplit("/", 1)[-1], file_content, loading=True)

        self.notify_function = notify_function

    def get_api(self, name):
        try:
            with open("{}/{}.py".format(config.APIS_PATH, name), "r") as f:
                file_content = f.read()
            return file_content
        except IOError as e:
            if "No such file or directory" in e:
                loginfo(e.message)
                return None
            else:
                raise IOError(e)

    def add_api(self, name, file_content, loading=False):
        loginfo("Adding api %s" % name)

        if name.endswith(".py"):
            name = name[:-3]
        file_name = name + ".py"

        loginfo("File name: ".format(file_name))

        loginfo("   applying Pylint...")
        lint_output = ""

        try:
            # todo maybe use lint
            loginfo("   compiling...")
            compile(file_content, file_name, "exec")
        except Exception as e:
            return False, traceback.format_exc(limit=0)
        loginfo("   compiled!")

        with open(config.APIS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        if not loading:
            options = """%s --msg-template='{path}({line:3d}:{column:2d}): [{obj}] {msg}' 
                --disable=C,R,I""" % (config.APIS_PATH + "/" + file_name)
            out, err = lint.py_run(options, return_std=True)
            lint_output = "".join(err.readlines()) + "".join(out.readlines())
            loginfo("lint says:")
            loginfo(lint_output)

        try:
            loginfo("   importing...")
            exec """import apis.{0} as {0}""".format(name)
            imported_api = eval(name)
            imp.reload(imported_api)
        except Exception as e:
            logerror(traceback.format_exc())
            return False, lint_output + traceback.format_exc(limit=1)
        loginfo("   imported!")

        api_funcs = [
            {
                "name": f[0],
                "doc": util.get_doc(imported_api),
                "api": name,
                "args": inspect.getargspec(f[1]).args
            }
            for f in
            [
                f for f in inspect.getmembers(imported_api, predicate=inspect.isfunction)
                if f[1].__module__ == imported_api.__name__ and not f[0].startswith("_")
            ]
        ]
        api = {"name": name, "doc": util.get_doc(imported_api)}

        loginfo("   adding funcs: {}".format(api_funcs))
        self.function_storage.delete_many({"api": name})
        self.api_storage.delete_many({"name": name})

        loginfo("   Funcs deleted. Adding now.")
        self.function_storage.insert_many(api_funcs)
        self.api_storage.insert(api)

        try:
            self.notify_function()
        except AttributeError as e:
            logwarn("No notify function set. If you see this message on startup, all is fine.")
        return True, lint_output

    def get_all_apis(self):
        return [api_path.rsplit("/", 1)[-1].rsplit(".py", 1)[-2] for api_path in self.get_all_api_files()]

    def get_all_api_files(self):
        """

        :rtype: list
        """
        api_files = []
        for (dirpath, _, file_names) in os.walk(config.APIS_PATH):
            api_files.extend([dirpath + "/" + x for x in file_names if x.endswith(".py") and not x.startswith(
                "__")])
        return api_files


def main():
    rospy.init_node("api_handler")
    loginfo("Creating api handler...")
    notify_publisher = rospy.Publisher("/aide/update_apis", Bool, queue_size=50)

    api = ApiHandler(lambda: notify_publisher.publish(True))
    loginfo("Registering services...")

    get_service_handler(GetApi).register_service(lambda **args: api.get_api(**args) or ())
    get_service_handler(GetAllApis).register_service(api.get_all_apis)
    get_service_handler(AddApi).register_service(api.add_api)

    loginfo("Registered services. Spinning.")

    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    loginfo(os.getcwd())
    rospy.spin()


if __name__ == "__main__":
    main()
