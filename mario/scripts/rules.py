import logging
import math
import os
import re
from abc import ABCMeta, abstractproperty, abstractmethod

import scipy.spatial
from enum import Enum
from pyparsing import ParseException
from rdflib.graph import ConjunctiveGraph
from rdflib.namespace import RDF, Namespace, RDFS
from rdflib.plugins.sparql import prepareQuery
from rdflib.plugins.sparql.processor import processUpdate
from rdflib.term import Literal, URIRef, BNode

from rdf_constants import BOOLEAN, FACE, SKEL_TRACKER, TYPES
from rdf_constants import id_from_face
from config import NAMESPACES_PATH, NAMESPACES_FORMAT

module_logger = logging.getLogger("mario.rules")


def _get_init_namespaces():
    g = ConjunctiveGraph()
    g.parse(NAMESPACES_PATH, format=NAMESPACES_FORMAT)
    return namespaces_as_dict(g)


def namespaces_as_dict(g):
    result = {}
    for row in g.namespaces():
        result[str(row[0])] = Namespace(row[1])
    return result


Namespaces = type('Namespaces', (unicode,), _get_init_namespaces())


class RuleHandler(ConjunctiveGraph):
    """
    This class manages the loading and execution of all defined rules which
    operate on the RDF graph.
    """

    def __init__(self, path, api=None, initial_onthology=None, format="turtle"):
        super(RuleHandler, self).__init__('Sleepycat')
        self.graph = self
        self.open(path, create=(not os.path.isdir(path)))
        if initial_onthology:
            self.parse(initial_onthology, format=format)
        self._cleanup_functions = set()
        self._on_add_functions = set()
        self._on_remove_functions = set()
        self._rules = []
        self.api = api
        self.logger = logging.getLogger("mario.rules.RuleHandler")

        self._namespaces_as_dict = lambda: namespaces_as_dict(self)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._cleanup()
        self.graph.close()

    def add_cleanup_function(self, func):
        self._cleanup_functions.add(func)

    def _cleanup(self):
        for func in self._cleanup_functions:
            func()

    def add_on_add_listener(self, func):
        self._on_add_functions.add(func)

    def add_on_remove_listener(self, func):
        self._on_add_functions.add(func)

    def add(self, triple_or_quad):
        for func in self._on_add_functions:
            func(triple_or_quad)
        super(RuleHandler, self).add(triple_or_quad)

    def remove(self, triple_or_quad):
        for func in self._on_remove_functions:
            func(triple_or_quad)
        super(RuleHandler, self).remove(triple_or_quad)

    def load_all_rules(self):
        """
        Loads all rules that inherit from the Rule abstract baseclass.
        """
        for SubRule in PeriodicRule.__subclasses__():
            rule = SubRule(self.graph)
            self._rules.append(rule)
            # try:
            #     self.graph.add_cleanup_function(rule.cleanup)
            # except AttributeError:
            #     print("{} has no cleanup Function.".format(rule.name))
            self.graph.add_cleanup_function(rule.cleanup)

        for SubRule in ReactiveRule.__subclasses__():
            print(SubRule)
            rule = SubRule(self.graph)
            self.graph.add_on_add_listener(rule.on_add)

    def execute_rules(self):
        """
        Executes all loaded rules.
        """
        for rule in [rl for rl in self._rules if not rl.executed]:
            # print("Executing rule {}".format(rule.name))
            rule.execute()

    def print_rules(self):
        """
        Prints the name of all loaded rules.
        """
        for rule in self._rules:
            print(rule.name)

    ALL_CLASSES_QUERY = prepareQuery(
        """SELECT ?class_name ?class_label WHERE {
        ?class_name rdf:type rdf:Class.
        ?class_name rdfs:label ?class_label}"""
    )

    def get_all_classes(self):
        return self.query(self.ALL_CLASSES_QUERY)

    ALL_PROPERTIES_FOR_CLASS_QUERY = prepareQuery(
        """
        SELECT ?name ?label ?domain ?range WHERE {
        ?name rdfs:label ?label.
        ?name rdfs:domain ?domain.
        ?name rdfs:range ?range.
        ?queried_class rdfs:subClassOf* ?domain.}
        """
    )

    def get_all_properties(self, class_name):
        return self.query(self.ALL_PROPERTIES_FOR_CLASS_QUERY,
                          initBindings={"queried_class": self._uri_from_class(
                              class_name)})

    CLASS_QUERY = prepareQuery(
        """
        SELECT ?queried_name ?class_label WHERE {
        ?queried_name rdfs:label ?class_label.}
        """
    )

    def get_class(self, class_name):
        return self.query(
            self.CLASS_QUERY,
            initBindings={"queried_name": self._uri_from_class(class_name)})

    def _uri_from_class(self, class_name):
        return URIRef(Namespaces.classes + "" + class_name)

    PPRINT_QUERY = prepareQuery("""
                CONSTRUCT {?s ?p ?o} WHERE {?s ?p ?o.
                MINUS {?s a rdf:Class.}
                MINUS {?s a rdf:Property.}}""")

    def pprint(self):
        res = self.query(self.PPRINT_QUERY)
        res.graph.namespace_manager = self.namespace_manager
        print(res.graph.serialize(format="n3"))

    ALL_RULES_QUERY = prepareQuery(
        """
        SELECT ?name ?description ?content WHERE {
        ?s a classes:rule.
        ?s rdfs:label ?name.
        ?s properties:content ?content.
        ?s properties:description ?description.}
        """, initNs=Namespaces.__dict__
    )

    RULE_QUERY = prepareQuery(
        """
        SELECT ?name ?content ?description WHERE {
        ?s rdfs:label ?name.
        ?s properties:content ?content.
        ?s properties:description ?description.}
        """, initNs=Namespaces.__dict__
    )

    def get_all_rules(self):
        return self.query(self.ALL_RULES_QUERY)

    def get_rule(self, rule_name):
        self.logger.debug("Getting " + rule_name)
        result = self.query(self.RULE_QUERY, initBindings={"name": Literal(rule_name)})
        size = len(result)
        self.logger.debug("Result has {} rows.".format(size))
        if size > 1:
            raise RuntimeError("Got two rules with same name. Something somewhere went terribly wrong.")
        return None if not result else next(result.__iter__())

    def save_and_create_rule(self, name, description, query):
        """

        :type name: str
        """
        if re.search("\s", name) is not None:
            raise ValueError("Name must not contain whitespaces!")
        rule_node = Namespaces.rules[name]

        if self.get_rule(name):
            # remove rule from rules list if there is one (actual python code)
            self._rules.remove((a for a in self._rules if a.name == name).next())
            # remove rule from graph (stored information about rule)
            self.remove((rule_node, None, None))

        language = Language.lang_from_content(query)

        rule = self._create_periodic_rule_from_query(name, query, language)
        try:
            rule.execute()
        except ParseException:
            return False

        self._rules.append(rule)

        self.add((rule_node, RDFS.label, Literal(name)))
        self.add((rule_node, RDF.type, Namespaces.classes.rule))
        self.add((rule_node, Namespaces.properties.content, Literal(query)))
        self.add((rule_node, Namespaces.properties.description, Literal(description)))
        return True

    def _create_periodic_rule_from_query(self, rule_name, query, language):
        """

        :type language: Language
        :type query: str
        :type rule_name: str
        """
        print language

        class GenericPeriodicRule(PeriodicRule):
            name = rule_name

            def __init__(self, graph):
                super(GenericPeriodicRule, self).__init__(graph)
                self.executed = False
                self.name = rule_name

            if language == Language.SPARQL:
                def execute(self):
                    result = self.graph.query(query)
                    for row in result:
                        self.graph.add(row)

            elif language == Language.UPDATE:
                def execute(self):
                    processUpdate(self.graph, query,
                                  initNs=self.graph._namespaces_as_dict())

            elif language == Language.EXECUTE:
                def execute(self):
                    construct = query.replace("EXECUTE", "CONSTRUCT")
                    result = self.graph.query(construct)
                    func_name = ""
                    args = {}
                    for s, p, o in result:
                        if p == Namespaces.functions.func:
                            func_name = o.toPython()
                        else:
                            args[p.toPython()[p.rindex('/') + 1:]] = o.toPython()
                    getattr(self.graph.api, func_name)(**args)
            elif language == Language.EXECUTE_BULK:
                def execute(self):

                    # split query to header and where clause (condition)
                    (header, where_clause) = query.split("WHERE")

                    # find all function names to be executed
                    func_names = re.findall("EXECUTE?\((.*?)\)", query)

                    for func_name in func_names:

                        # select = re.sub("EXECUTE?\((.*?)\)", "SELECT", query)
                        args = [arg for arg in re.findall("{}?\)(.*?)[\n]".format(func_name), query)[0].split(" ") if
                                arg]
                        if not args:
                            select = "SELECT ?dummy_arg WHERE {{ {} BIND(3 as ?dummy_arg) }}".format(where_clause)
                        else:
                            select = "SELECT {} WHERE {}".format(" ".join(args), where_clause)

                        result = self.graph.query(select)

                        for row in result:
                            row_as_dict = row.asdict()

                            print("Calling {}".format(func_name))
                            if not args:
                                print("with no args")
                                func_args = {}
                            else:
                                func_args = {x: row_as_dict[x].toPython() for x in row_as_dict}
                                print("with args: " + str(args))
                            print("All Conditions true, executing rule" + self.name)
                            self.graph.api.call(func_name, **func_args)
                            self.executed = True
            else:
                raise ValueError("Unknown Language!")

        return GenericPeriodicRule(self.graph)

    def add_function(self, func_text):
        return self.api.add_function(self.api.assemble_function(func_text))


class PeriodicRule:
    """
    This is the abstract base class for all rules which operate on the RDF
    graph.

    In order for a rule to be loaded, it has to inherit this class and also
    at least implement a name attribute and an execute method. The execute
    method is called periodically at the moment. Optionally, the rule can
    add a cleanup function which is called on graph closing. (i.e. to
    remove
    parts of the graph which are non-persistent)
    """
    __metaclass__ = ABCMeta

    def __init__(self, graph):
        """
        Creates the rule.

        Should be called by all inheriting rules.
        :type graph: RuleHandler
        """
        self.graph = graph

    @abstractproperty
    def name(self):
        """
        Descriptive name of the rule.

        Must be implemented by any rule.
        """
        pass

    @abstractmethod
    def execute(self):
        """
        Executes the rule.

        Must be implemented by any rule.
        """
        pass

    def cleanup(self):
        """
        Function to be called on graph closing.

        Optional.
        """
        print("{} has nothing to clean up.".format(self.name))


class ReactiveRule:
    __metaclass__ = ABCMeta

    def __init__(self, graph):
        """
        Creates the rule.

        Should be called by all inheriting rules.
        :type graph: scripts.rdf.RdfGraph
        """
        self.graph = graph

    def on_add(self, triple_or_quad):
        if self.condition(triple_or_quad):
            self.action(triple_or_quad)

    @abstractmethod
    def condition(self, triple_or_quad):
        pass

    @abstractmethod
    def action(self, triple_or_quad):
        pass


class SamplePeriodicRule(PeriodicRule):
    """
    Some sample rule that just prints all tracked faces and their
    positions.
    """
    name = "print_tracked_faces_positions"
    query = prepareQuery(
        """
        SELECT ?face_id ?x ?y ?z WHERE {
        ?face_id faces:tracked boolean:true.
        ?face_id faces:position_x ?x.
        ?face_id faces:position_y ?y.
         ?face_id faces:position_z ?z.
        }
        """,
        initNs={"faces": FACE, "boolean": BOOLEAN})

    def execute(self):
        result = self.graph.query(self.query)
        for row in result:
            print("face_id: {} @ ({:.2f},{:.2f},{:.2f})".format(*tuple(
                x.toPython() for x in row)))


class SkeletonLinker(PeriodicRule):
    name = "skeleton_linker"

    unlinked_faces_query = prepareQuery(
        """
        SELECT ?face_id ?x ?y ?z WHERE {
        ?face_id faces:tracked boolean:true.
        ?face_id faces:position_x ?x.
        ?face_id faces:position_y ?y.
        ?face_id faces:position_z ?z.
        FILTER NOT EXISTS {?face_id skeleton:skeleton ?id}
        }
        """,
        initNs={"faces"   : FACE, "boolean": BOOLEAN,
                "skeleton": SKEL_TRACKER})

    unlinked_skeletons_query = prepareQuery(
        """
        SELECT ?skel_id ?x ?y ?z WHERE {
        ?skel_id skeleton:tracked boolean:true.
        ?skel_id skeleton:position_x ?x.
        ?skel_id skeleton:position_y ?y.
        ?skel_id skeleton:position_z ?z.
        FILTER NOT EXISTS {?face_id skeleton:skeleton ?skel_id}
        }
        """
        , initNs={"faces"   : FACE, "boolean": BOOLEAN,
                  "skeleton": SKEL_TRACKER})

    def near(self, position1, position2):
        try:
            distance = scipy.spatial.distance.euclidean(position1,
                                                        position2)
            print(
                "Skeleton Position: ({:.2f},{:.2f},{:.2f})".format(
                    *position2))
            print(distance)
        except ValueError:
            return False
        return distance <= 0.15

    def execute(self):
        # get all unlinked faces
        unlinked_faces_result = self.graph.query(self.unlinked_faces_query)
        # get all unlinked skeletons
        unlinked_skeletons_result = self.graph.query(
            self.unlinked_skeletons_query)
        # if there are both unlinked faces and unlinked skeletons
        if (len(unlinked_faces_result) > 0 and len(
                unlinked_skeletons_result) > 0):
            for face_row in unlinked_faces_result:
                face_position = tuple(x.toPython() for x in face_row[1:])
                for skeleton_row in unlinked_skeletons_result:
                    skeleton_position = tuple(x.toPython() for x in
                                              skeleton_row[1:])
                    if (self.near(face_position, skeleton_position)):
                        # if coordinates considered near then link face to
                        # skeleton
                        self.graph.add((face_row[0],
                                        SKEL_TRACKER.skeleton,
                                        skeleton_row[0]))


class TooNearPeriodicRule(PeriodicRule):
    """
    A rule that notifies when some tracked face is too near to be tracked.
    """
    name = "too_near_rule"
    query = prepareQuery(
        """
        SELECT ?face_id ?x ?y ?z WHERE {
        ?face_id faces:tracked boolean:true.
        ?face_id faces:position_x ?x.
        ?face_id faces:position_y ?y.
        ?face_id faces:position_z ?z.
        }
        """,
        initNs={"faces": FACE, "boolean": BOOLEAN})

    def execute(self):
        result = self.graph.query(self.query)
        for row in result:
            if any(math.isnan(x.toPython()) for x in tuple(row)[1:]):
                print("Head back!")


class SampleGreeterRule(ReactiveRule):
    name = "sample_greeter_rule"

    def condition(self, triple_or_quad):
        # Regard only tracking events
        if not (triple_or_quad[1] == FACE.tracked and triple_or_quad[2] == BOOLEAN.true):
            return False
        # Regard only tracking events that are new
        if (triple_or_quad) in self.graph:
            return False
        return True

    def action(self, triple_or_quad):
        self.graph.api.say("Hello {}".format(id_from_face(
            triple_or_quad[0].toPython())))


class ResetFaceTrackRule(ReactiveRule):
    name = "reset_face_track_rule"

    def condition(self, triple_or_quad):
        # Regard only speech events
        if not (triple_or_quad[1] == TYPES.speech):
            return False
        return Literal("reset") == triple_or_quad[2]

    def action(self, triple_or_quad):
        self.graph.api.reset_face_tracking()


class Language(Enum):
    SPARQL = Namespaces.classes.sparql.toPython()
    UPDATE = Namespaces.classes.update.toPython()
    EXECUTE = Namespaces.classes.execute.toPython()
    EXECUTE_BULK = Namespaces.classes.executeBulk.toPython()

    @staticmethod
    def convert(name):
        module_logger.info("Converting {} to {}".format(name, Language[name]))
        try:
            return Language[name]
        except KeyError:
            raise ValueError("Unknown Name")

    @staticmethod
    def lang_from_content(content):
        return Language.EXECUTE_BULK if content.startswith("EXECUTE") else Language.SPARQL