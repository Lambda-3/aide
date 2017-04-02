import logging
import threading

from flask import Flask
from flask_restful import Api, Resource, reqparse, marshal, abort
from flask_restful import fields

module_logger = logging.getLogger("mario.rest")


class RestApi:
    graph = None

    def __init__(self, graph):
        """

        :type graph: scripts.rules.RuleHandler
        """
        self.app = Flask("asdf", static_url_path="")
        self.api = Api(self.app)
        RestApi.graph = graph
        self.graph.print_rules()
        self.logger = logging.getLogger("mario.rest.RestApi")
        self.logger.info("Creating an instance of RestApi.")

        # Adding resources
        self.api.add_resource(self.RulesResource, '/mario/rules',
                              endpoint='rules')
        self.api.add_resource(self.RuleResource, '/mario/rule/<int:id>',
                              endpoint='rule')
        self.api.add_resource(self.ClassesResource, '/mario/classes',
                              endpoint='classes')
        self.api.add_resource(self.ClassResource,
                              '/mario/classes/<string:class_name>',
                              endpoint='class')
        self.api.add_resource(self.PropertiesResource,
                              '/mario/classes/<string:class_name>/properties',
                              endpoint='properties')
        self.api.add_resource(self.FunctionsResource, '/mario/functions',
                              endpoint='functions')
        self.api.add_resource(self.NameSpacesResource, '/mario/namespaces',
                              endpoint='namespaces')

    def run(self):
        self.process_in_bg = threading.Thread(target=self.app.run,
                                              kwargs={'host': '0.0.0.0'})
        self.process_in_bg.daemon = True
        self.process_in_bg.start()

    def debug(self):
        self.app.run(debug=True)

    rdf_class_fields = {
        'URI': fields.String(attribute="class_name"),
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

    @staticmethod
    def rdflib_to_restful(result_set):
        result = []
        fields = result_set.vars
        for row in result_set:
            dict_row = dict()
            for i in range(len(row)):
                dict_row[str(fields[i])] = row[i].toPython()
            result.append(dict_row)
        if len(result) == 1:
            return result[0]
        else:
            return result

    @staticmethod
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
        parser.add_argument('language', required=True, type=str,
                            choices=(RestApi.graph.CLASSES.sparql.toPython(),
                                     RestApi.graph.CLASSES.update.toPython(),
                                     RestApi.graph.CLASSES.execute.toPython(),
                                     RestApi.graph.CLASSES.executeBulk.toPython()),
                            help="Only SPARQL and SPARQL Update "
                                 "are allowed.")
        return parser.parse_args()

    @staticmethod
    def parse_class():
        parser = reqparse.RequestParser()
        parser.add_argument('name', required=True, type=str,
                            help="Class needs a name!")
        parser.add_argument('subClassOf', required=False, default=None,
                            type=str)

    @staticmethod
    def parse_property():
        parser = reqparse.RequestParser()
        parser.add_argument('name', required=True, type=str,
                            help="Property needs a name!")
        parser.add_argument('domain', required=True, type=str,
                            help="Property needs a domain (class of subject)!")
        parser.add_argument('range', required=True, type=str,
                            help="Property needs a range (class of object)!")

    class RulesResource(Resource):
        def get(self):
            return True

        def post(self):
            args = RestApi.parse_rule()
            result = RestApi.graph.save_and_create_rule(
                args.name,
                args.content,
                args.language,
                args.description,
                type=RestApi.graph.CLASSES.periodicRule)
            if not result:
                abort(400, message="Query could not be parsed!")
            return result

    class RuleResource(Resource):
        def get(self, id):
            return True

        def delete(self, id):
            return True

    class ClassesResource(Resource):
        def get(self):
            query_result = RestApi.graph.get_all_classes()
            return marshal(RestApi.rdflib_to_restful(query_result),
                           RestApi.rdf_class_fields, envelope="classes")

        def post(self):
            return True

    class ClassResource(Resource):
        def get(self, class_name):
            query_result = RestApi.graph.get_class(class_name)

            return marshal(RestApi.rdflib_to_restful(query_result),
                           RestApi.rdf_class_fields)

    class PropertiesResource(Resource):
        def get(self, class_name):
            query_result = RestApi.graph.get_all_properties(class_name)
            return marshal(
                RestApi.rdflib_to_restful(query_result),
                RestApi.rdf_property_fields)

        def post(self):
            return True

    class FunctionsResource(Resource):
        def get(self):
            return True

    class NameSpacesResource(Resource):
        def get(self):
            namespaces = RestApi.graph.namespaces()
            result = []
            for ns in namespaces:
                dict_row = dict()
                dict_row['namespace'] = ns[0]
                dict_row['URI'] = ns[1]
                result.append(dict_row)
            return marshal(result, RestApi.namespace_fields)


if __name__ == '__main__':
    RestApi(None).debug()
