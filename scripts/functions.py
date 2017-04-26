#!/usr/bin/env python

import re
import rospy
import roslib
import pymongo
from mario.msg import Function
from mario.srv._GetFunction import GetFunctionResponse

roslib.load_manifest("mario")

from ros_services import get_service_handler
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as rtd
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as dtr
from rospy_message_converter.json_message_converter import convert_json_to_ros_message as jtr

from rospy import loginfo


class SimpleApi:
    def __init__(self):
        self.db = pymongo.MongoClient().db
        self.db.functions.create_index("name", unique=True)
        self.functions_table = self.db.functions
        self.load_all()

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
        try:
            self.__dict__[funct.__name__]
            raise ValueError("Function already exists!")
        except KeyError:
            pass

        self.__dict__[funct.__name__] = funct.__get__(self, type(self))
        return True

    @staticmethod
    def build_function(function_text):
        name = re.findall("def (\w+)", function_text)[0]
        rospy.loginfo("Building function with name {}".format(name))
        # !!! IN NO WAY IS THIS SECURE !!!
        exec (function_text)
        return locals()[name]

    def add_from_text(self, name, doc, content, new=True, **kwargs):
        success = self.add_function(self.build_function(content))
        if success and new:
            self.store(name, doc, content)
        return success

    def store(self, name, doc, content):
        self.functions_table.insert_one({"name": name, "doc": doc, "content": content})

    def load(self, name):
        return self.functions_table.find_one({"name": name})

    def load_all(self):
        for row in self.functions_table.find():
            self.add_from_text(new=False, **row)

    def get(self, name):
        result = self.functions_table.find_one({"name": name})
        return result


def main():
    rospy.init_node("functions")
    api = SimpleApi()
    get_service_handler("AddFunction").register_service(
        lambda req: api.add_from_text(req.function.name, req.function.doc, req.function.content))

    def get_function(req):
        result = api.get(req.name)
        try:
            del result['_id']
            return GetFunctionResponse(function=dtr("mario/Function", result))
        except:
            return None

    get_service_handler("GetFunction").register_service(get_function)
    rospy.spin()


if __name__ == "__main__":
    main()
