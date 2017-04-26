#!/usr/bin/env python
import threading
import roslib
import rospy

roslib.load_manifest("mario")

from rospy import loginfo, logdebug
from flask import Flask
from flask_cors import CORS
from flask_restful import Api, fields, reqparse, Resource, abort, marshal
from enum import Enum

from mario.msg import Function
from ros_services import get_service_handler

rdf_class_fields = {
    'URI': fields.String(attribute="queried_name"),
    'label': fields.String(attribute="class_label")
}

rdf_property_fields = {
    'URI': fields.String(attribute="name"),
    'label': fields.String,
    'domain': fields.String,
    'range': fields.String
}

namespace_fields = {
    'URI': fields.String,
    'namespace': fields.String
}


# rules
def save_rule(name, description, content):
    add = get_service_handler('AddRule').get_service()
    add(name, description, content)


def get_rule(name):
    get = get_service_handler('GetRule').get_service()
    return get()


def get_all_rules():
    get = get_service_handler('GetAllRules').get_service()
    return get()


rule_fields = {
    'name': fields.String,
    'description': fields.String,
    'language': fields.String,
    'content': fields.String
}

rule_fields_short = {
    'name': fields.String,
    'description': fields.String
}


# Functions
def save_function(name, doc, content):
    add = get_service_handler('AddFunction').get_service()
    return add(Function(name, doc, content)).success


def get_function(name):
    get = get_service_handler('GetFunction').get_service()
    return get(name).function


def get_all_functions():
    # get = get_service_handler('GetAllFunctions').get_service()
    # return get()
    return []


function_fields = {
    'name': fields.String,
    'doc': fields.String,
    'content': fields.String
}


def parse_function():
    parser = reqparse.RequestParser()
    parser.add_argument('name', required=True, type=str,
                        help="Function needs a name!")
    parser.add_argument('doc', required=False, type=str, default="",
                        help="Provide documentation to your function!")
    parser.add_argument('content', required=True, type=str,
                        help="Need function content!")
    return parser.parse_args()


class ResourceEnum(Enum):
    Functions = {"post": save_function,
                 "get": get_all_functions,
                 "marshal_with": None,
                 "envelope": None,
                 "validate_input": parse_function,
                 }

    Function = {"get": get_function,
                "marshal_with": function_fields,
                "path": ("/mario/functions/<string:name>",)
                }

    def validate_input(self):
        return self.value['validate_input']()

    def has_get(self):
        return True if self.value.has_key("get") else False

    def get(self, *args, **kwargs):
        return self.value["get"](*args, **kwargs)

    def has_post(self):
        return True if self.value.has_key("post") else False

    def post(self, *args, **kwargs):
        return self.value["post"](*args, **kwargs)

    def path(self):
        if self.value.has_key("path"):
            return self.value["path"]
        else:
            name = self.name.lower()
            return ("/mario/{}".format(name), "/mario/{}/".format(name))

    def envelope(self):
        return self.value["envelope"] if self.value.has_key("envelope") else None

    def marshal_with(self):
        return self.value["marshal_with"] if self.value.has_key("marshal_with") else None


def build_resources(api):
    """
    This function builds the api resources generically based on the entries in ResourceEnum.
    
    The resources are then added to a given API.
    :param api: API for which the resources are to be built.
    :return: Nothing
    """
    for definition in ResourceEnum:
        # definition is the resource definition from the enum based on it's dictionary.
        name = definition.name.lower()
        loginfo("Initializing for {}".format(name))

        # Class dictionary of the Resource
        class_dict = {}

        if definition.has_post():
            loginfo("{} has post, creating post function".format(name))

            def post(self, rsc=definition):
                """
                Post method of the resource to be created.
                
                :param self: Reference to the object this function is later bound to.
                :param rsc: Never touch this! This is for early binding purposes only.
                :return: 
                """
                args = rsc.validate_input()
                try:
                    result = rsc.post(**args)
                    loginfo("Success, Result: " + str(result))
                    return result
                except rospy.ServiceException as e:
                    error = e.message.split("error processing request: ")[1]
                    loginfo("Unsuccessful, got following error: " + error)
                    abort(400, message=error)

            class_dict["post"] = post

        if definition.has_get():
            loginfo("{} has get, creating get function".format(name))

            def get(self, rsc=definition, *args, **kwargs):
                """
                Get method of the resource to be created.
                
                :param self: Reference to the object this function is later bound to.
                :param rsc: Never touch this! This is for early binding purposes only.
                :param args: Positional arguments, given through to the defined get function.
                :param kwargs: Keyword arguments, given through to the defined get function.
                :return: 
                """
                try:
                    result = rsc.get(*args, **kwargs)
                    if result:
                        return marshal(result, rsc.marshal_with(), rsc.envelope())

                except rospy.ServiceException as e:
                    if e.message.endswith("None"):
                        abort(404)
                    error = e.message.split("error processing request: ")[1]
                    loginfo("Unsuccessful, got following error: " + error)
                    abort(400, message=error)

            class_dict["get"] = get

        # Create the class dynamically with previously defined functions
        GeneratedResource = type(str(definition.name), (Resource,), class_dict)
        loginfo("Created Resource {} with endpoint = {}".format(GeneratedResource, name))

        # Add generated resource to the api
        api.add_resource(GeneratedResource, *definition.path(), endpoint=name)


class RestApi:
    def __init__(self):
        loginfo("Creating RestApi instance.")
        self.app = Flask("Mario", static_url_path="")
        self.api = Api(self.app)
        CORS(self.app)
        build_resources(self.api)

    def run(self):
        self.process_in_bg = threading.Thread(target=self.app.run,
                                              kwargs={'host': '0.0.0.0'})
        self.process_in_bg.daemon = True
        self.process_in_bg.start()

    def debug(self):
        self.app.run(debug=True)


def main():
    rospy.init_node("rest")
    api = RestApi()
    api.run()
    rospy.spin()


if __name__ == '__main__':
    main()
