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

from mario.msg import Function, Rule
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
    return get(name)


def get_all_rules():
    get = get_service_handler('GetAllRules').get_service()
    return get()


rule_fields_short = {
    'name': fields.String,
    'description': fields.String
}


def parse_rule():
    parser = reqparse.RequestParser()
    parser.add_argument('name', required=True, type=str,
                        help="Rule needs a name!",
                        location="json")
    parser.add_argument('content', required=True, type=str,
                        help="Rule needs content!",
                        location="json")
    parser.add_argument('description', required=False, type=str,
                        default="No description available.",
                        location="json")
    return parser.parse_args()


# Functions
def save_function(name, doc, content):
    return get_service_handler('AddFunction').get_service()(Function(name, doc, content)).success


def get_function(name):
    get = get_service_handler('GetFunction').get_service()
    return get(name).function


def get_all_functions():
    # get = get_service_handler('GetAllFunctions').get_service()
    # return get()
    return []


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
    Functions = {
        "post": lambda **args: get_service_handler('AddFunction').get_service()(Function(**args)).success,
        "marshal_with": None,
        "envelope": None,
        "validate_input": parse_function,
    }

    Function = {
        "get": lambda **args: get_service_handler('GetFunction').get_service()(**args).function,
        "marshal_with": {
            'name': fields.String,
            'doc': fields.String,
            'content': fields.String
        },
        "path": "/mario/functions/<string:name>"
    }
    Rules = {
        "post": lambda **args: get_service_handler('AddRule').get_service()(Rule(**args)).success,
        "get": lambda: get_service_handler('GetAllRules').get_service()().rules,
        "marshal_with": {
            'name': fields.String,
            'description': fields.String
        },
        "envelope": "rules",
        "validate_input": parse_rule,
    }

    Version = {
        "get": lambda: {"version": '"Alluring Alliteration"'},
        "marshal_with": None,
        "envelope": None,
        "path": "/mario/_version"
    }

    Rule = {
        "get": lambda **args: get_service_handler('GetRule').get_service()(**args).rule,
        "marshal_with": {
            'name': fields.String,
            'description': fields.String,
            'content': fields.String
        },
        "envelope": "rule",
        "path": "/mario/rules/<string:name>"
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
            path = self.value["path"]
            # if path is a tuple just return it, if not add a trailing slash if there is none.
            return path if isinstance(path, tuple) else (path, path + "/") if not path.endswith("/") else (path,)
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
                        try:
                            return marshal(result, rsc.marshal_with(), rsc.envelope())
                        except AttributeError as e:
                            if "'NoneType' object has no attribute 'items'" in e.message:
                                return result
                            else:
                                raise AttributeError(e)


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
