#!/usr/bin/env python
import json

import roslib
import rospy
from rdflib.term import URIRef

from sparql_completer import QueryCompleter

roslib.load_manifest('mario')

import config

from rospy import loginfo, logdebug
from ros_services import get_service_handler
from mario_messages.srv import GetAllRules, GetRule, AddRule, CallFunction
from mario_messages.msg import RdfTriple
from rules import RuleHandler


def pythonize_row(row):
    """
    
    :param row: Row to be converted to a dict.
    :type row: rdflib.query.ResultRow
    :return: 
    :rtype: dict
    """
    return {k: v.toPython() for k, v in row.asdict().items()}


def pythonize(result_set):
    """
    Converts a rdflib result set to a list of dicts which represent the rows of the result set.
    
    :param result_set: Result set to convert
    :type result_set: rdflib.plugins.sparql.processor.SPARQLResult
    :return: A list of dicts which represent the rows of a result
    :rtype: list
    """
    return [pythonize_row(row) for row in result_set]


class ROSStub:
    def call(self, _func_name, **kwargs):
        loginfo("Calling function {} with kwargs: ".format(_func_name, json.dumps(kwargs)))
        get_service_handler(CallFunction).get_service()(_func_name, json.dumps(kwargs))


def main():
    rospy.init_node("rdf")
    with RuleHandler(config.RDF_PATH, api=ROSStub(), initial_onthology=config.ONTHOLOGY_PATH) as cep_and_kb:
        # delete everything to not pollute the graph
        cep_and_kb.parse(config.SCRIPTS_PATH + "/BackgroundKnowledgeGraph.rdf", format="n3")
        cep_and_kb.add_cleanup_function(lambda: cep_and_kb.remove((
            URIRef("http://prokyon:5000/mario/rules/mrs_smith_fell"), None, None)))
        cep_and_kb.add_cleanup_function(lambda: cep_and_kb.remove(
            (None, None, URIRef("http://prokyon:5000/mario/fell"))))
        cep_and_kb.add_cleanup_function(lambda: cep_and_kb.remove(
            (None, None, None)))

        def create_triple_from_msg(msg):
            """
            Callback function to execute when a triple is published on the channel.
            
            :type msg: RdfTriple
            """
            result = cep_and_kb.query(
                """CONSTRUCT {{ {} {} {} }} WHERE {{ }}""".format(msg.subject, msg.predicate, msg.object))

            for row in result:
                cep_and_kb.set(row)
                cep_and_kb.pprint()

        rospy.Subscriber("/mario/rdf", RdfTriple, create_triple_from_msg)

        get_service_handler(AddRule).register_service(
            lambda rule: cep_and_kb.save_and_create_rule(rule.name, rule.description, rule.content))

        get_service_handler(GetRule).register_service(lambda name: pythonize_row(cep_and_kb.get_rule(name)))

        get_service_handler(GetAllRules).register_service(lambda: pythonize(cep_and_kb.get_all_rules()))
        cep_and_kb.pprint()
        while not rospy.is_shutdown():
            cep_and_kb.execute_rules()
            rospy.sleep(3)


if __name__ == "__main__":
    main()
