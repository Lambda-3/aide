#!/usr/bin/env python
import json

import roslib
import rospy
from mario.srv._GetAllRules import GetAllRulesResponse

roslib.load_manifest('mario')

import config

from rospy import loginfo, logdebug
from ros_services import get_service_handler
from mario.srv import GetRuleResponse
from mario.msg import RdfTriple
from rules import RuleHandler

from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as dtr


def rdflib_to_dict(result_set):
    loginfo("processing " + str(result_set))
    result = []
    fields = result_set.vars
    loginfo("bound vars: " + str(result_set.vars))

    for row in result_set:
        dict_row = dict()
        for i in range(len(row)):
            dict_row[str(fields[i])] = str(row[i])
        result.append(dict_row)
    if len(result) == 1:
        return result[0]
    else:
        return result


class ROSStub:
    def call(self, name, **kwargs):
        loginfo("Calling function {} with kwargs: ".format(name, json.dumps(kwargs)))
        get_service_handler("CallFunction").get_service()(name, json.dumps(kwargs))


def main():
    rospy.init_node("rdf_handler")
    with RuleHandler(config.RDF_PATH, api=ROSStub(), initial_onthology=config.ONTHOLOGY_PATH) as cep_and_kb:
        # delete everything to not pollute the graph
        cep_and_kb.add_cleanup_function(lambda: cep_and_kb.remove((None, None, None)))

        def create_triple_from_msg(msg):
            """

            :type msg: RdfTriple
            """
            result = cep_and_kb.query(
                """CONSTRUCT {{ {} {} {} }} WHERE {{ }}""".format(msg.subject, msg.predicate, msg.object))

            for row in result:
                cep_and_kb.set(row)

        rospy.Subscriber("/mario/rdf", RdfTriple, create_triple_from_msg)

        get_service_handler("AddRule").register_service(
            lambda req: cep_and_kb.save_and_create_rule(req.rule.name,
                                                        req.rule.description,
                                                        req.rule.content))

        def get_rule(req):
            result = cep_and_kb.get_rule(req.name)
            loginfo("Got Result {}".format(result))
            if result:
                return GetRuleResponse(dtr("mario/Rule", rdflib_to_dict(result)))
            else:
                return None

        get_service_handler("GetRule").register_service(get_rule)

        def get_all_rules(req):
            result = cep_and_kb.get_all_rules()
            loginfo("Got {} rules".format(len(result)))
            rule_list = []
            dict_result = rdflib_to_dict(result)
            if len(result) == 1:
                return GetAllRulesResponse([dtr("mario/Rule", dict_result)])
            for row in dict_result:
                loginfo("Appending {}".format(row))
                rule_list.append(dtr("mario/Rule", row))
            loginfo(type(rule_list))
            return GetAllRulesResponse(rules=rule_list)

        get_service_handler("GetAllRules").register_service(get_all_rules)
        while not rospy.is_shutdown():
            print len(cep_and_kb.get_all_rules())
            cep_and_kb.execute_rules()
            cep_and_kb.pprint()
            rospy.sleep(3)


if __name__ == "__main__":
    main()
