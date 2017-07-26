import rospy
from aide_messages.msg import RdfTripleStamped, RdfGraphStamped
from rdflib import Literal, XSD, Namespace

from rdflib.term import URIRef

# namespaces TODO: do it more generically
speech = Namespace("http://lambda3.org/aide/speech/")
properties = Namespace("http://lambda3.org/aide/properties/")
robot = Namespace("http://lambda3.org/aide/self/")

def Graph(*triples):
    return RdfGraphStamped(triples)


def Triple(subject, predicate, object, time=None):
    """

    :type predicate: rdflib.term.URIRef
    :type subject: rdflib.term.URIRef
    """
    return RdfTripleStamped(subject.toPython(), predicate.toPython(), literalize(object), time or rospy.Time.now())


def literalize(literal):
    if isinstance(literal, URIRef):
        return literal.toPython()
    literal = Literal(literal)
    datatype = literal.datatype or XSD.string
    return "{}^^{}".format(literal, datatype)
