#!/usr/bin/env python

import imp
import inspect
import os
import traceback

import roslib
import rospy
from aide_messages.srv import AddApi, GetAllApis, GetApi
from pylint import epylint as lint
from rospy import loginfo
from rospy.core import logerror, logwarn
from std_msgs.msg import Bool

from aide_core import config
from apis import storage, util
from apis.ros_services import get_service_handler

roslib.load_manifest("aide_core")


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

        try:
            loginfo("   compiling...")
            compile(file_content, file_name, "exec")
        except Exception:
            return False, traceback.format_exc(limit=0)
        loginfo("   compiled!")

        with open(config.APIS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        lint_output = ""
        if not loading:
            loginfo("   applying Pylint...")
            lint_output = util.apply_lint(config.APIS_PATH + "/" + file_name)

        loginfo("   importing...")
        try:
            exec ("""import apis.{0} as {0}""".format(name))
            imported_api = eval(name)
            imp.reload(imported_api)
        except Exception:
            logerror(traceback.format_exc())
            return False, lint_output + traceback.format_exc(limit=1)
        loginfo("   imported!")

        api_funcs = [
            {
                "name": f[0],
                "doc": util.get_doc(imported_api),
                "api": name,
                "args": inspect.getargspec(f[1]).args,
                "hinted_args": util.get_type_hints(f[1])[0]
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
        except AttributeError:
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
