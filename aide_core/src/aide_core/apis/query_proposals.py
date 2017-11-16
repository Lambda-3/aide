import re
from itertools import groupby

import rdflib
import rospy
from aide_messages.msg import RdfGraphStamped
from rdflib import RDF, RDFS
from rospy import loginfo
from rospy.core import logerror

from aide_core.apis.util import to_space
from aide_core.apis.rdf_utils import properties, string_to_literal
from rdflib.plugins.sparql.processor import prepareQuery
from rdflib.term import URIRef, Literal

from aide_core.apis import indra


class QueryProposal(object):
    def __init__(self, cls, property, range, single=False, entity=None):
        self.cls = cls
        self.explicit_class = not "genClass" in cls.toPython()
        self.single = single if single else False
        if self.single:
            self.entity = entity
        if not self.explicit_class:
            self.extractor = re.findall("_(.*)", self.cls.rsplit("genClass", 1)[-1])[0]
        self.property = property
        self.range = range

    def format_for_indra(self, include_extractor=True):
        formatted_cls = self.cls.rsplit("/", 1)[-1]
        if not self.explicit_class:
            formatted_cls = formatted_cls.split("genClass")[-1].strip()
            if not include_extractor:
                formatted_cls = formatted_cls[1:].split("_")[-1]

        formatted_prop = self.property.rsplit("/", 1)[-1].strip()

        return to_space("{0} {1}".format(formatted_cls, formatted_prop))

    def __repr__(self):
        return repr(self.format_for_sparql())

    def format_for_sparql(self):
        result = []
        lit_name = "?{}".format(self.range.rsplit("XMLSchema#")[1])
        if self.explicit_class:
            result.append("?s a {}".format(self.cls))
        result.append("?s {prop} {lit}".format(prop=self.property, lit=lit_name))
        return result

    def get_sparql(self):
        result = []
        subj_name = "?s" if not self.single else self.entity
        try:
            lit_name = "?{}{}".format(self.property.rsplit("/",1)[1], self.range.rsplit("XMLSchema#")[1].capitalize())
        except IndexError:
            lit_name = "?{}".format(self.property.rsplit("/",1)[-1])
        if self.explicit_class:
            result.append({
                "subject": subj_name,
                "predicate": "a",
                "object": self.cls
            })
        result.append({
            "subject": subj_name,
            "predicate": self.property.toPython(),
            "object": lit_name
        })
        return result


class QueryProposalManager(object):
    property_path = """
    SELECT DISTINCT ?domain ?prop ?range ?single ?entity WHERE {
        ?prop rdfs:range ?range.
        OPTIONAL {
                ?prop rdfs:domain ?domain.
            }
        OPTIONAL {
                ?domain properties:single ?single;
                    properties:instance ?entity.
            }
        }
    """

    with_classes = """
    SELECT DISTINCT ?domain ?prop ?range ?single ?entity WHERE {
        ?prop rdfs:range ?range.
        VALUES ?domain {%s}
            OPTIONAL {
                ?prop rdfs:domain ?domain.
            }
        OPTIONAL {
                ?domain properties:single ?single;
                    properties:instance ?entity.
            }
        }
    """

    from_entity = """
    SELECT DISTINCT ?domain ?prop ?range ?single ?entity WHERE {
        ?prop rdfs:range ?range.
        ?prop rdfs:domain ?domain.
        ?entity a ?domain;
            properties:single ?single.    
        }
    """

    subject_and_class_query = """
    SELECT DISTINCT ?instance ?class WHERE {
        ?instance a ?class.
        }
    """

    domains_query = """
    SELECT DISTINCT ?domain WHERE {
        ?prop rdfs:domain ?domain.
        }
    """
    # bind ?class and ?subject
    single_query = """
    ASK { 
    FILTER NOT EXISTS{
        ?class properties:single false
        } 
    FILTER NOT EXISTS{
        ?class properties:instance ?entity 
        FILTER(?entity != ?subject)
        }
    }"""

    single_instance_query = """"""

    def __init__(self):
        self.graph = rdflib.ConjunctiveGraph()
        # TODO
        self.graph.parse("/home/viktor/workspace/catkin_ws/src/aide/aide_core/Namespaces.rdf", format="turtle")
        ns = dict(self.graph.namespaces())
        print self.graph.serialize(format="n3")
        self.property_path = prepareQuery(self.property_path, initNs=ns)
        self.subject_and_class_query = prepareQuery(self.subject_and_class_query, initNs=ns)
        self.domains_query = prepareQuery(self.domains_query, initNs=ns)
        self.from_entity = prepareQuery(self.from_entity, initNs=ns)
        self.single_query = prepareQuery(self.single_query, initNs=ns)
        rospy.Subscriber("/aide/rdf", RdfGraphStamped, self.extend_onthology)
        # TODO: for the time being
        # self.graph.parse(config.PROJECT_PATH + "/ObservedDataGraph.rdf", format="turtle")

    def extend_onthology(self, incoming_graph):
        """

        :type incoming_graph: RdfGraphStamped
        """
        pending_ranges = []
        subjects_with_classes = dict()
        grouped_by_subject = (grouped for grouped in groupby(incoming_graph.quadruples, lambda x: x.subject))
        for subject, triples in grouped_by_subject:
            subject = URIRef(subject)
            # triples = [triple for triple in triples]
            # loginfo("subject: {} TRIPLES: {}".format(subject, [triple for triple in triples]))
            # classes = [triple.object for triple in triples if triple.predicate == RDF.type.toPython()]
            # loginfo("classes: {}".format(classes))
            # predicates = [(triple.predicate, triple.object) for triple in triples if triple.predicate !=
            #               RDF.type.toPython()]
            classes, predicates = [], []
            for triple in triples:
                predicate = URIRef(triple.predicate)
                if predicate == RDF.type:
                    classes.append(URIRef(triple.object))
                else:
                    predicates.append((predicate, string_to_literal(triple.object)))
            # loginfo("predicates: {}".format(predicates))

            subjects_with_classes[subject] = classes
            if not "uuid-" in subject:
                # if 'uuid' in subject URI, then its a candidate for single
                for cls in classes:
                    loginfo("class: {}".format(cls))
                    loginfo("subject: {}".format(subject))
                    result = self.graph.query(self.single_query, initBindings={"class": cls, "subject":
                        subject}).askAnswer
                    loginfo("Is single?: {}".format(result))
                    if result:
                        self.graph.addN([
                            (cls, properties.single, Literal(True), self.graph),
                            (cls, properties.instance, subject, self.graph)
                        ])
                    else:
                        self.graph.set((cls, properties.single, Literal(False)))
            props = []
            for predicate, object in predicates:
                # set domains
                props.extend((predicate, RDFS.domain, cls, self.graph) for cls in classes)
                # set ranges
                if isinstance(object, Literal):
                    # if range is literal, easy
                    props.append((predicate, RDFS.range, object.datatype, self.graph))
                elif isinstance(object, URIRef):
                    # if not, check if subject is known at the end
                    pending_ranges.append((predicate, object))
                else:
                    e = RuntimeError("Something that's neither a URI nor a literal detected in RDF stream.")
                    logerror(e)
                    raise e
            self.graph.addN(props)

        # lastly check for pending ranges if class is known now
        pending_props = []
        for predicate, object in pending_ranges:
            classes = (subjects_with_classes.get(object, None))
            if classes:
                pending_props.extend((URIRef(predicate), RDFS.range, cls, self.graph) for cls in classes)
        self.graph.addN(pending_props)
        print self.graph.serialize(format="n3")

    def get_query_proposals_from_plaintext(self, plain_text, subj_classes=None, entity=None, top_k=5):
        if (isinstance(subj_classes, str)):
            subj_classes = [subj_classes]
        if subj_classes:
            query = self.with_classes % (" ".join("<{}>".format(cls) for cls in subj_classes))
            result = self.graph.query(query)
        elif entity:
            result = self.graph.query(self.from_entity, initBindings={"entity": URIRef(entity)})
        else:
            result = self.graph.query(self.property_path)
        query_proposals = []
        for row in result:
            print(row)
            query_proposals.append(QueryProposal(*row))

        result = indra.sort_by_relatedness(to_space(plain_text), query_proposals, repr_function='format_for_indra')

        return [x.get_sparql() for x in result][:top_k]

    def get_domains_of_property(self, property):
        result = self.graph.query(self.domains_query, initBindings={"prop": URIRef(property)})
        return [x[0].toPython() for x in result]


_qp = None


def __init():
    global _qp
    _qp = QueryProposalManager()


def get_query_proposal(text, classes=None, entity=None, top_k=5):
    """
    Top level interface function.

    Gets `top_k` SPARQL statements sorted by their semantic relatedness to `string`. The statements are derived from:

    - data observed on the rdf channel.

    - TODO: explicit ontology definitions in the extractors

    - TODO: implicit onthology derived from code analysis

    :param string:
    :type string: str
    :return:
    :rtype: str
    """
    if not _qp:
        __init()
    return _qp.get_query_proposals_from_plaintext(text, classes, entity, top_k)


def get_domains_of_property(property):
    if not _qp:
        __init()
    return _qp.get_domains_of_property(property)
