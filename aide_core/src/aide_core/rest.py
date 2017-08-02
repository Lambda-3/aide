#!/usr/bin/env python
import threading

import roslib
import rospy
from aide_messages.msg import Routine, Event
from aide_messages.srv import (AddRule, GetApi, GetAllApis, GetActionProvider, GetAllActionProviders,
                               AddActionProvider, AddExtractor, GetAllExtractors, GetExtractor, GetAllRoutines,
                               GetEvent, GetAllEvents, AddApi, GetRoutine)
from enum import Enum
from flask import Flask
from flask_cors import CORS
from flask_restful import Api, reqparse, Resource, abort, marshal
from rospy import loginfo, logdebug

from aide_core.apis import query_proposals
from apis import approximate
from apis.ros_services import get_service_handler
from apis.util import camel_case_to_underscore, format_code
from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as to_ros_message
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as to_dict

roslib.load_manifest("aide_core")


def parse_api():
    parser = reqparse.RequestParser()

    parser.add_argument('name', required=True, type=str,
                        help="Api needs name!",
                        location="json")
    parser.add_argument('file_content', required=True, type=str,
                        help="must not be empty api!",
                        location="json")

    return parser.parse_args()


def parse_code():
    parser = reqparse.RequestParser()
    parser.add_argument('code', required=True, type=str,
                        help="Code must not be empty!",
                        location="json")

    return parser.parse_args()


def parse_sparql_autocomplete():
    parser = reqparse.RequestParser()
    parser.add_argument('text', required=True, type=str,
                        help="Text must not be empty!",
                        location="json")
    parser.add_argument('classes', required=False, type=list,
                        default=None, location="json")
    parser.add_argument('top_k', required=False, type=int, default=5, location="json")
    return parser.parse_args()


def parse_rule():
    # TODO
    # subject of change, will be replaced by a generalized function
    # that validates json input against a ros message
    def parse_routine(routine):
        return to_ros_message(Routine._type, routine)

    def parse_events(events):
        parsed_events = []
        for event in events:
            parsed_events.append(to_ros_message(Event._type, event))
        return parsed_events

    parser = reqparse.RequestParser()
    parser.add_argument('routine', required=True,
                        type=parse_routine,
                        help="Need a routine!",
                        location="json")
    parser.add_argument('events', required=False, type=parse_events, default=[], location="json")
    return parser.parse_args()


class ResourceEnum(Enum):
    ActionProvider = {
        "get": get_service_handler(GetActionProvider).call_service,
        "path": "/aide/action_providers/<string:name>"
    }
    ActionProviders = {
        "post": get_service_handler(AddActionProvider).call_service,
        "get": get_service_handler(GetAllActionProviders).call_service,
        "validate_input": parse_api
    }

    Extractor = {
        "get": get_service_handler(GetExtractor).call_service,
        "path": "/aide/extractors/<string:name>"
    }
    Extractors = {
        "post": get_service_handler(AddExtractor).call_service,
        "get": get_service_handler(GetAllExtractors).call_service,
        "validate_input": parse_api
    }

    Api = {
        "get": get_service_handler(GetApi).call_service,
        "path": "/aide/apis/<string:name>"
    }
    Apis = {
        "post": get_service_handler(AddApi).call_service,
        "get": get_service_handler(GetAllApis).call_service,
        "validate_input": parse_api
    }

    Routine = {
        "get": get_service_handler(GetRoutine).call_service,
        "path": "/aide/routines/<string:name>"
    }

    Routines = {
        "get": get_service_handler(GetAllRoutines).call_service,
    }

    Event = {
        "get": get_service_handler(GetEvent).call_service,
        "path": "/aide/events/<string:name>"
    }

    Events = {
        "get": get_service_handler(GetAllEvents).call_service
    }

    Format = {
        "post": format_code,
        "validate_input": parse_code,
        "envelope": "formatted_code"
    }

    Rules = {
        "post": lambda **args: get_service_handler(AddRule).call_service(**args),
        "validate_input": parse_rule,
    }

    Related = {
        "get": lambda **args: approximate.get_semantically_related(**args),
        'path': "/aide/<string:type>/<string:to>/related/<int:top_k>",
        "envelope": "result"
    }

    Version = {
        "get": lambda: {"version": '"Alluring Alliteration"'},
        "path": "/aide/_version"
    }

    SparqlComplete = {
        "path": "/aide/qp",
        "post": lambda **args: query_proposals.get_query_proposal(**args),
        "validate_input": parse_sparql_autocomplete,
        "envelope": "proposals"

    }

    #
    # ClassProposals = {
    #     "path"        : ("/aide/classes/<string:subject>/related/<int:top_k>",
    #                      "/aide/classes/<string:subject>/related/"),
    #     "get"         : (lambda subject, top_k=3: completer.get_subject_from_plaintext(subject)),
    #     "marshal_with": {
    #         "instance": fields.String,
    #         "class"   : fields.String
    #     },
    #     "envelope"    : "classes"
    # }

    def validate_input(self):
        return self.value['validate_input']()

    def has_get(self):
        return "get" in self.value

    def get(self, *args, **kwargs):
        return self.value["get"](*args, **kwargs)

    def has_post(self):
        return "post" in self.value

    def post(self, *args, **kwargs):
        return self.value["post"](*args, **kwargs)

    def path(self):
        """

        :rtype: tuple
        """
        if "path" in self.value:
            path = self.value["path"]
            # if path is a tuple just return it, if not add a trailing slash if there is none and convert to tuple.
            return path if isinstance(path, tuple) else (path, path + "/") if not path.endswith("/") else (path,)
        else:
            # if no path value is defined, create a default path
            name = camel_case_to_underscore(self.name)
            return "/aide/{}".format(name), "/aide/{}/".format(name)

    def envelope(self):
        # return self.value["envelope"] if self.value.has_key("envelope") else None
        return self.value.get("envelope", None)

    def marshal_with(self):
        return self.value.get("marshal_with", None)


def handle_result(result, rsc):
    logdebug(result)
    if result or result == []:
        try:
            return marshal(result, rsc.marshal_with(), rsc.envelope())
        except AttributeError as e:
            if "'NoneType' object has no attribute 'items'" in str(e):
                loginfo(e)
                try:
                    result = to_dict(result)
                    loginfo(result)
                    return marshal(result, rsc.marshal_with(), rsc.envelope())
                except AttributeError as e:
                    if isinstance(result, dict):
                        logdebug("result is already dict")
                        return result
                    else:
                        loginfo(str(e))
                        return {rsc.envelope(): result}

            else:
                raise AttributeError(e)


def handle_exception(e):
    if str(e).endswith("None"):
        loginfo(e)
        abort(404)
    error = str(e).split("error processing request: ")[1]
    loginfo("Unsuccessful, got following error: " + error)
    abort(400, message=error)


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
                kwargs = rsc.validate_input()
                loginfo("POST request on {} with args {}".format(self.__class__, str(kwargs)))
                try:
                    result = rsc.post(**kwargs)
                    return handle_result(result, rsc)
                except rospy.ServiceException as e:
                    handle_exception(e)

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
                loginfo("GET request on {} with args {}".format(self.__class__, str(kwargs)))
                try:
                    result = rsc.get(*args, **kwargs)
                    return handle_result(result, rsc)
                except rospy.ServiceException as e:
                    handle_exception(e)

            class_dict["get"] = get

        # Create the class dynamically with previously defined functions
        GeneratedResource = type(str(definition.name), (Resource,), class_dict)
        loginfo("Created Resource {} with endpoint = {}".format(GeneratedResource, name))

        # Add generated resource to the api
        api.add_resource(GeneratedResource, *definition.path(), endpoint=name)


class RestApi:
    def __init__(self):
        loginfo("Creating RestApi instance.")
        self.app = Flask("aide", static_url_path="")
        self.api = Api(self.app, catch_all_404s=True)
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
