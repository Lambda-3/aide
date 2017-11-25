import rospy
from aide_messages.msg import RdfTripleStamped, RdfGraphStamped
from rdflib import Literal, XSD, Namespace

from rdflib.term import URIRef
from rospy.core import logerror, loginfo

properties = Namespace("http://lambda3.org/aide/properties/")
classes = Namespace("http://lambda3.org/aide/classes/")


def Graph(*args):
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

    :type predicate: rdflib.term.URIRef
    :type subject: rdflib.term.URIRef
    """
    return RdfTripleStamped(subject.toPython(), predicate.toPython(), literal_to_string(object),
                            time or rospy.Time.now())


def literal_to_string(literal):
    if isinstance(literal, URIRef):
        return literal.toPython()
    literal = Literal(literal)
    datatype = literal.datatype or XSD.string
    return "{}^^{}".format(literal, datatype)


def string_to_literal(string, to_uri=True):
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
