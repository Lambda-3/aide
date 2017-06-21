import rospy
from mario_messages.msg import RdfTripleStamped, RdfGraphStamped
from rdflib import Literal, XSD, Namespace

from rdflib.term import URIRef

# namespaces TODO: do it more generically
speech = Namespace("http://prokyon:5000/mario/speech/")
properties = Namespace("http://prokyon:5000/mario/properties/")
mario = Namespace("http://prokyon:5000/mario/")

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
