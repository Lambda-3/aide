"""
System API. Don't change if you don't know what you're doing.
"""

import rospy
from aide_messages.msg import RdfTripleStamped, RdfGraphStamped
from rdflib import Literal, XSD, Namespace

from rdflib.term import URIRef
from rospy.core import logerror, loginfo

properties = Namespace("http://lambda3.org/aide/properties/")
classes = Namespace("http://lambda3.org/aide/classes/")


def Graph(*args):
    """
    Creates a graph from given tuple of subjects.
    :param args: tuple of subjects
    :return: Graph message.
    """
    list_of_subjects = all(getattr(r, "_is_subject", False) for r in args)
    if list_of_subjects:
        loginfo("Is a list of subjects")
        triples_list = [triple for tpl in args for triple in tpl.to_rdf_triples()]
        loginfo(type(triples_list[0]))
        return RdfGraphStamped(triples_list)
    else:
        loginfo("not a list of subjects!")
    try:
        return RdfGraphStamped(args)

    except TypeError as e:
        logerror(str(e))

        return None


def Triple(subject, predicate, object, time=None):
    """
    Creates a triple message from given subject, predicate and object
    :type predicate: rdflib.term.URIRef
    :type subject: rdflib.term.URIRef
    :rtype RdfTripleStamped
    """
    return RdfTripleStamped(subject.toPython(), predicate.toPython(), literal_to_string(object),
                            time or rospy.Time.now())


def literal_to_string(literal):
    """
    Converts a given literal to its string representation ("literal^^xsd:type"). Does nothing if literal is a
    URIRef object.
    :param literal: literal in the sense of rdf.
    :return: formatted literal in form "literal^^xsd:type".
    """
    if isinstance(literal, URIRef):
        return literal.toPython()
    literal = Literal(literal)
    datatype = literal.datatype or XSD.string
    return "{}^^{}".format(literal, datatype)


def string_to_literal(string, to_uri=True):
    """
    Creates a literal or URI object from a given string.
    :param string: String to convert from
    :param to_uri: Whether to try convert to an URI if not a valid literal is given.
    :return: Literal object or URI object (if failed to create a literal and desired) or else input string.
    """
    if "^^http://www.w3.org/2001/XMLSchema#" in string:
        try:
            value, datatype = string.split("^^")
            return Literal(value, datatype=datatype)
        except:
            loginfo("Could not deliteralize string.")
            return Literal(string)
    else:
        loginfo("not a literal")
        return URIRef(string) if to_uri else string
