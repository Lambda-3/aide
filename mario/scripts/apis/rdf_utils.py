import rospy
from mario_messages.msg import RdfTripleStamped
from mario_messages.msg._RdfGraphStamped import RdfGraphStamped
from rdflib import Literal, XSD


def Graph(*triples):
    return RdfGraphStamped(triples)


def Triple(subject, predicate, object, time=None):
    return RdfTripleStamped(subject, predicate, literalize(object), time or rospy.Time.now())


def literalize(literal):
    if ":" in literal:
        return literal
    literal = Literal(literal)
    datatype = literal.datatype or XSD.string
    return "{}^^{}".format(literal, datatype)
