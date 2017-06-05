#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Bool

roslib.load_manifest("mario")
import json

import os
import inspect
import traceback
import imp
import apis
import requests
import pymongo

from bson import ObjectId
from mario_messages.srv._GetApi import GetApi
from rospy.core import logerror, logwarn

from mario_messages.srv import GetSemRelatedFunctions, AddApi, GetAllApis

import config

from ros_services import get_service_handler

from rospy import loginfo


class ApiHandler:
    def __init__(self, notify_function):
        self.db = pymongo.MongoClient().db
        self.db.functions_api.create_index("name", unique=True)
        self.functions_api_table = self.db.functions_api
        # self.apis_table = self.db.api
        self.db.apis.create_index("name", unique=True)

        # for the time being just load out of the folder /apis folder
        api_files = self.get_all_api_files()

        for file_name in api_files:
            loginfo("Working on: {}".format(file_name))
            with open(file_name, "r") as f:
                file_content = f.read()
            self.add_api(f.name.rsplit("/", 1)[-1], file_content)

        self.notify_function = notify_function

    def store_func_in_db(self, name, doc, args, api):
        self.functions_api_table.insert_one({"name": name, "doc": doc, "args": args, "api": api})

    def get_function(self, name):
        result = self.functions_api_table.find_one({"name": name}, {"_id": False})
        return result

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

    def add_api(self, name, file_content):
        # class_name = re.findall("class\s+(.*?)[\(,:]", file_content)[0]
        # api_name = cc_to_underscore(api_name)
        loginfo("Adding api %s" % name)
        if name.endswith(".py"):
            name = name[:-3]
        file_name = name + ".py"
        loginfo("File name: ".format(file_name))
        try:
            # todo maybe use lint
            loginfo("   compiling...")
            compile(file_content, file_name, "exec")
        except Exception as e:
            return (False, traceback.format_exc(limit=0))

        loginfo("   compiled!")
        with open(config.APIS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        try:
            loginfo("   importing...")
            exec """import apis.{0} as {0}\nimp.reload({0})""".format(name)
            # ApiClass = eval("{}.{}".format(api_name, class_name))
            imported_api = eval(name)
        except Exception as e:
            logerror(traceback.format_exc())
            return (False, traceback.format_exc(limit=1))

        loginfo("   imported!")
        api_funcs = [{"name": f[0], "doc": f[1].__doc__ or "", "api": name, "args": inspect.getargspec(f[1]).args}
                     for f in [f for f in inspect.getmembers(imported_api, predicate=inspect.isfunction) if f[
                1].__module__ == imported_api.__name__]]
        loginfo("   adding funcs: {}".format(api_funcs))
        self.functions_api_table.delete_many({"api": name})
        loginfo("   Funcs deleted. Adding now.")
        self.functions_api_table.insert_many(api_funcs)
        # setattr(self, api_name, ApiClass())
        try:
            self.notify_function()
        except AttributeError as e:
            logwarn("No notify function set. If you see this message on startup, all is fine.")
        return (True, "")

    # def call_func(self, func_name, **kwargs):
    #     api, func = func_name.rsplit(".", 1)
    #     try:
    #         new_kwargs = dict()
    #         for k, v in kwargs.items():
    #             try:
    #                 new_kwargs[k] = eval("self." + v)
    #             except:
    #                 new_kwargs[k] = v
    #         getattr(eval("self.{}".format(api)), func)(**new_kwargs)
    #     except KeyError:
    #         raise ValueError("There is no function {} in api {}!".format(func, api))

    def get_all_apis(self):
        return [api_path.rsplit("/", 1)[-1].rsplit(".py", 1)[-2] for api_path in self.get_all_api_files()]

    def get_all_api_files(self):
        """

        :rtype: generator
        """
        api_files = []
        for (dirpath, _, file_names) in os.walk(config.APIS_PATH):
            api_files.extend([dirpath + "/" + x for x in file_names if x.endswith(".py") and not x.startswith(
                "__")])
        return api_files

    def get_best_matches(self, name, top_k):

        """

        :rtype: list
        """
        # build pairs from given name and every func + their api in database. use id to later identify
        pairs = [{"t1": name.replace("_", " "),
                  "t2": "id={id} {api} {name}".format(id=row['_id'], name=row['name'].replace("_", " "),
                                                      api=row['api'].rsplit("_")[0])}
                 for row in self.functions_api_table.find({}, {"name": True, "api": True, "_id": True})]

        data = {'corpus'       : 'wiki-2014',
                'model'        : 'W2V',
                'language'     : 'EN',
                'scoreFunction': 'COSINE', 'pairs': pairs}

        headers = {
            'content-type': "application/json"
        }

        response = requests.request("POST", "http://localhost:8916/relatedness", data=json.dumps(data), headers=headers)

        response.raise_for_status()

        response = response.json()['pairs']

        index = top_k if len(response) > top_k else len(response)

        return [self.functions_api_table.find_one({"_id": ObjectId(rsp['t2'].split(" ", 1)[0].split("=")[1])},
                                                  {"_id": False})
                for rsp in sorted(response, reverse=True)][:index]


def main():
    rospy.init_node("api_handler")
    loginfo("Creating api handler...")
    notify_publisher = rospy.Publisher("/mario/update_apis", Bool, queue_size=50)

    api = ApiHandler(lambda: notify_publisher.publish(True))
    loginfo("Registering services...")

    get_service_handler(GetSemRelatedFunctions).register_service(api.get_best_matches)
    get_service_handler(GetApi).register_service(api.get_api)
    get_service_handler(GetAllApis).register_service(api.get_all_apis)
    get_service_handler(AddApi).register_service(api.add_api)
    loginfo("Registered services. Spinning.")

    rospy.spin()


if __name__ == "__main__":
    main()
