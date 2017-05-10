#!/usr/bin/env python
import threading
import roslib
import rospy
import config
from mario.srv import GetSemRelatedFunctions, AddFunction, GetAllFunctions, GetFunction, AddRule, GetAllRules, GetRule
from rdflib.term import URIRef

from sparql_completer import QueryCompleter

roslib.load_manifest("mario")

from rospy import loginfo, logdebug
from flask import Flask
from flask_cors import CORS
from flask_restful import Api, fields, reqparse, Resource, abort, marshal
from enum import Enum

from mario.msg import Function, Rule
from ros_services import get_service_handler

rdf_class_fields = {
    'URI'  : fields.String(attribute="queried_name"),
    'label': fields.String(attribute="class_label")
}

rdf_property_fields = {
    'URI'   : fields.String(attribute="name"),
    'label' : fields.String,
    'domain': fields.String,
    'range' : fields.String
}

namespace_fields = {
    'URI'      : fields.String,
    'namespace': fields.String
}

rule_fields_short = {
    'name'       : fields.String,
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


def parse_function():
    parser = reqparse.RequestParser()
    parser.add_argument('name', required=True, type=str,
                        help="Function needs a name!")
    parser.add_argument('doc', required=False, type=str, default="",
                        help="Provide documentation to your function!")
    parser.add_argument('args', required=False, type=str, default="",
                        help="Arguments for the function!", action='append')
    parser.add_argument('body', required=True, type=str,
                        help="Need function body!")
    args = parser.parse_args()
    loginfo(args.args)
    return args


# This might be moved somewhere eventually
completer = QueryCompleter(config.SCRIPTS_PATH + "/FancyOnthologyGraph.rdf", config.SCRIPTS_PATH +
                           "/BackgroundKnowledgeGraph.rdf")


class ResourceEnum(Enum):
    Function = {
        "get"         : lambda **args: get_service_handler(GetFunction).get_service()(**args).function,
        "marshal_with": {
            'name': fields.String,
            'doc' : fields.String,
            'args': fields.List(fields.String),
            'body': fields.String,
        },
        "path"        : "/mario/functions/<string:name>"
    }

    Functions = {
        "post"          : lambda **args: get_service_handler(AddFunction).get_service()(Function(**args)).success,
        "get"           : lambda: get_service_handler(GetAllFunctions).get_service()().functions,
        "marshal_with"  : Function["marshal_with"],
        "envelope"      : "functions",
        "validate_input": parse_function,
    }

    ReladedFunctions = {d: v for d, v in Functions.items() if d is not "post"}  # copy everything but post
    ReladedFunctions['get'] = (lambda top_k=3, **args:
                               get_service_handler(GetSemRelatedFunctions).get_service()(top_k=top_k, **args).functions)
    ReladedFunctions['path'] = ("/mario/functions/<string:name>/related/<int:top_k>",
                                "/mario/functions/<string:name>/related/")

    Rules = {
        "post"          : lambda **args: get_service_handler(AddRule).get_service()(Rule(**args)).success,
        "get"           : lambda: get_service_handler(GetAllRules).get_service()().rules,
        "marshal_with"  : {
            'name'       : fields.String,
            'description': fields.String
        },
        "envelope"      : "rules",
        "validate_input": parse_rule,
    }

    Version = {
        "get"         : lambda: {"version": '"Alluring Alliteration"'},
        "marshal_with": None,
        "envelope"    : None,
        "path"        : "/mario/_version"
    }

    QueryProposals = {
        "path"        : ("/mario/classes/<string:subject_class>/qp/<string:plain_text>/<int:top_k>",
                         "/mario/classes/<string:subject_class>/qp/<string:plain_text>"),
        "get"         : (lambda subject_class, plain_text, top_k=3:
                         completer.get_queries_from_plaintext_and_subject(
                             plain_text=plain_text, top_k=top_k,
                             subject_class=URIRef("http://prokyon:5000/mario/classes/" + subject_class))),

        "marshal_with": {
            "code": fields.String,
            "path": fields.String
        },
        "envelope"    : "proposals"

    }

    ClassProposals = {
        "path"        : ("/mario/classes/<string:subject>/related/<int:top_k>",
                         "/mario/classes/<string:subject>/related/"),
        "get"         : (lambda subject, top_k=3: completer.get_subject_from_plaintext(subject)),
        "marshal_with": {
            "instance": fields.String,
            "class"   : fields.String
        },
        "envelope"    : "classes"
    }

    Rule = {
        "get"         : lambda **args: get_service_handler(GetRule).get_service()(**args).rule,
        "marshal_with": {
            'name'       : fields.String,
            'description': fields.String,
            'content'    : fields.String
        },
        "envelope"    : "rule",
        "path"        : "/mario/rules/<string:name>"
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
        """

        :rtype: tuple
        """
        if self.value.has_key("path"):
            path = self.value["path"]
            # if path is a tuple just return it, if not add a trailing slash if there is none and convert to tuple.
            return path if isinstance(path, tuple) else (path, path + "/") if not path.endswith("/") else (path,)
        else:
            # if no path value is defined, create a default path
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
                loginfo("POST request on {} with args {}".format(self.__class__, str(args)))
                try:
                    result = rsc.post(**args)
                    logdebug("Success, Result: " + str(result))
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
                    loginfo("GET request on {} with args {}".format(self.__class__, str(kwargs)))
                    result = rsc.get(*args, **kwargs)
                    loginfo(result)
                    if result or result == []:
                        try:
                            return marshal(result, rsc.marshal_with(), rsc.envelope())
                        except AttributeError as e:
                            if "'NoneType' object has no attribute 'items'" in e.message:
                                loginfo(e)
                                return result
                            else:
                                raise AttributeError(e)
                except rospy.ServiceException as e:
                    if e.message.endswith("None"):
                        loginfo(e)
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
