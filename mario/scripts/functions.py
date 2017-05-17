#!/usr/bin/env python
import json

import requests
import rospy
import roslib
import pymongo
import action_api as ros

from mario_messages.srv import AddFunction, CallFunction, GetFunction, GetAllFunctions, GetSemRelatedFunctions

roslib.load_manifest("mario")

from ros_services import get_service_handler
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as rtd

from rospy import loginfo


class SimpleApi:
    def __init__(self):
        self.db = pymongo.MongoClient().db
        self.db.functions.create_index("name", unique=True)
        self.functions_table = self.db.functions
        self.load_all()
        self.ros = ros

    def __getattr__(self, name):
        if not self.__dict__[name]:
            def dummy_func(*args, **kwargs):
                loginfo("Fake-Invoking function {}:".format(name))
                if args:
                    loginfo("Arguments: " + str(args))
                if kwargs:
                    loginfo("Keyword Arguments: " + str(kwargs))

            self.__dict__[name] = dummy_func

        return self.__dict__[name]

    def add_function(self, funct):
        rospy.loginfo("Adding function with name {}".format(funct.__name__))
        if self.__dict__.has_key(funct.__name__):
            raise ValueError("Function already exists!")
        else:
            pass

        self.__dict__[funct.__name__] = funct.__get__(self, type(self))
        return True

    @staticmethod
    def assemble_function(name, args, body):
        function_text = "def " + name + "(self," + ",".join(args) + "):"
        function_text += "\n  " + "\n  ".join(body.splitlines())

        rospy.loginfo("Assembled function with name {}".format(name))

        return function_text

    @staticmethod
    def build_function(function_text):
        # !!! IN NO WAY IS THIS SECURE !!!
        exec (function_text)
        locals().pop("function_text")
        return locals().popitem()[1]

    def add_from_text(self, name, doc, body, args, new=True):
        if name in ("function_text", "locals"):
            raise ValueError("Nice Try!")
        function_text = self.assemble_function(name, args, body)
        built_func = self.build_function(function_text)

        success = self.add_function(built_func)
        if success and new:
            self.store(name, doc, args, function_text)
        return success

    def store(self, name, doc, args, function_text):
        self.functions_table.insert_one({"name": name, "doc": doc, "args": args, "body": function_text})

    def load(self, name):
        return self.functions_table.find_one({"name": name}, {"_id": False})

    def load_all(self):
        for row in self.functions_table.find({}, {"_id": False}):
            func = self.build_function(row["body"])
            self.add_function(func)

    def get(self, name):
        result = self.functions_table.find_one({"name": name}, {"_id": False})
        return result

    def call(self, func_name, **kwargs):
        try:
            getattr(self, func_name)(**kwargs)
        except KeyError as e:
            raise ValueError("There is no function with such name!")

    def get_all_functions(self):
        """

        :rtype: generator
        """
        return self.functions_table.find({}, {"_id": False})

    def get_best_matches(self, name, top_k):

        """

        :rtype: list
        """
        # build pairs from given name and every func in database
        pairs = [{"t1": name.replace("_", " "), "t2": row['name'].replace("_", " ")} for row in
                 self.functions_table.find({}, {"name": True, "_id": False})]

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

        return [self.get(rsp['t2'].replace(" ", "_")) for rsp in sorted(response, reverse=True)][:index]


def main():
    rospy.init_node("functions")
    api = SimpleApi()

    get_service_handler(AddFunction).register_service(lambda function: api.add_from_text(**rtd(function)))

    get_service_handler(CallFunction).register_service(lambda func_name, kwargs: api.call(func_name, **json.loads(
        kwargs)))

    get_service_handler(GetFunction).register_service(api.get)

    get_service_handler(GetAllFunctions).register_service(lambda: [x for x in api.get_all_functions()])

    get_service_handler(GetSemRelatedFunctions).register_service(api.get_best_matches)

    rospy.spin()


if __name__ == "__main__":
    main()
