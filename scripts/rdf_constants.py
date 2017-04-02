from rdflib import Namespace, URIRef
from rdflib.plugins.sparql import prepareQuery

PATH = "/home/viktor/.rdf/mario"
LOGPATH = "../log/mario.log"
PERSONS = Namespace("mario.persons.")
FACE = Namespace("mario.persons.faces.")
TYPES = Namespace("mario.types.")
SKEL_TRACKER = Namespace("mario.skel_tracker.")
BOOLEAN = Namespace("mario.boolean.")
NAME_QUERY = prepareQuery(
    """
    SELECT ?name WHERE {
    ?id persons:name ?name.
    }
    """,
    initNs={"persons": PERSONS})
UUID_QUERY = prepareQuery(
    "SELECT ?id WHERE {?id persons:name ?queried_name}",
    initNs={"persons": PERSONS})
DISTANCE_QUERY = prepareQuery(
    """
    SELECT ?distance WHERE {
    ?skel_id skel_tracker:distance_to_me ?distance.
    ?id skel_tracker:skeleton ?skel_id.
    ?id persons:name ?queried_name.
    }
    """,
    initNs={"persons": PERSONS, "skel_tracker": SKEL_TRACKER})
TRACKED_PERSONS_QUERY = prepareQuery(
    """
    SELECT ?name WHERE {
    ?id persons:name ?name.
    ?id skel_tracker:skeleton ?skel_id .
    }
    """,
    initNs={"persons": PERSONS, "skel_tracker": SKEL_TRACKER})
TRACKED_FACES_QUERY = prepareQuery(
    """
    SELECT ?face_id WHERE {
    ?face_id faces:tracked boolean:true
    }
    """,
    initNs={"faces": FACE, "boolean": BOOLEAN})




def id_from_face(uri_ref):
    # type: (URIRef) -> str
    """
    This little helper returns the face/skeleton ID from an URIRef object.

    :type uri_ref: URIRef
    :return: ID as a str.
    """
    return str(uri_ref).split(".")[-1]


def face_from_id(face_id):
    """

    :type face_id: str
    """
    return URIRef("mario.persons.faces.ids.{}".format(face_id))


def skeleton_from_id(skel_id):
    """

    :type skel_id: int
    """
    return URIRef("mario.skel_tracker.ids.%d" % skel_id)
