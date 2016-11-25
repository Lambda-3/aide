import math
from abc import ABCMeta, abstractproperty, abstractmethod

from rdflib.plugins.sparql import prepareQuery

from rdf_constants import BOOLEAN, FACE


class RuleHandler:
    """
    This class manages the loading and execution of all defined rules which
    operate on the RDF graph.
    """

    def __init__(self, graph):
        """

        :type graph: scripts.rdf.RdfGraph
        """
        self._rules = []
        self.graph = graph

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
            pass

    def execute_rules(self):
        """
        Executes all loaded rules.
        """
        for rule in self._rules:
            rule.execute()

    def print_rules(self):
        """
        Prints the name of all loaded rules.
        """
        for rule in self._rules:
            print(rule.name)


class PeriodicRule:
    """
    This is the abstract base class for all rules which operate on the RDF
    graph.

    In order for a rule to be loaded, it has to inherit this class and also
    at least implement a name attribute and an execute method. The execute
    method is called periodically at the moment. Optionally, the rule can
    add a cleanup function which is called on graph closing. (i.e. to remove
    parts of the graph which are non-persistent)
    """
    __metaclass__ = ABCMeta

    def __init__(self, graph):
        """
        Creates the rule.

        Should be called by all inheriting rules.
        :type graph: scripts.rdf.RdfGraph
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
    Some sample rule that just prints all tracked faces and their positions.
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
        print("tracked faces:")
        result = self.graph.query(self.query)
        for row in result:
            print("face_id: {} @ ({:.2f},{:.2f},{:.2f})".format(*tuple(
                x.toPython() for x in row)))


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
        print("faces too near:")
        result = self.graph.query(self.query)
        for row in result:
            if any(math.isnan(x.toPython()) for x in tuple(row)[1:]):
                print("Head back!")
