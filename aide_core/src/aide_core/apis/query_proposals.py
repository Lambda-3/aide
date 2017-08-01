import re

import rdflib

from aide_core import config
from aide_core.apis.util import to_space
from rdflib.plugins.sparql.processor import prepareQuery
from rdflib.term import URIRef

from aide_core.apis import indra


class QueryProposal(object):
    def __init__(self, cls, property, range):
        cls_name = cls.rsplit("/", 1)[-1]
        self.cls = cls
        self.explicit_class = not "genClass" in cls.toPython()
        self.singleton = not re.match("(.*)(\d+)", cls_name) and not self.explicit_class
        if not self.explicit_class:
            self.namespace = re.findall("_(.*)_", self.cls.rsplit("genClass", 1)[-1])[0]
        self.property = property
        self.range = range

    def format_for_indra(self, include_namespaces=True):
        formatted_cls = self.cls.rsplit("/", 1)[-1]
        if not self.explicit_class:
            formatted_cls = re.sub("\d", "", formatted_cls.split("genClass")[-1]).strip()
            if not include_namespaces:
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
        subj_name = "?s" if not self.singleton else "{}:{}".format(*self.cls.rsplit("_")[-2:])
        lit_name = "?{}".format(self.range.rsplit("XMLSchema#")[1])
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
    SELECT DISTINCT ?domain ?prop ?range WHERE {
        ?prop rdfs:range ?range.
        OPTIONAL {
                ?prop rdfs:domain ?domain.
            }
        }
    """

    with_classes = """
    SELECT DISTINCT ?domain ?prop ?range WHERE {
        ?prop rdfs:range ?range.
        VALUES ?domain {%s}
            OPTIONAL {
                ?prop rdfs:domain ?domain.
            }
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

    def __init__(self):
        self.graph = rdflib.ConjunctiveGraph()

        # TODO: for the time being
        self.graph.parse(config.PROJECT_PATH + "/ObservedDataGraph.rdf", format="turtle")

        self.domains_query = prepareQuery(self.domains_query)

    def get_query_proposals_from_plaintext(self, plain_text, subj_classes=None, top_k=5):
        if (isinstance(subj_classes, str)):
            subj_classes = [subj_classes]
        if subj_classes:
            query = self.with_classes % (" ".join("<{}>".format(cls) for cls in subj_classes))
            result = self.graph.query(query)
        else:
            result = self.graph.query(self.property_path)
        query_proposals = []
        for row in result:
            query_proposals.append(QueryProposal(*row))

        result = indra.sort_by_relatedness(to_space(plain_text), query_proposals, repr_function='format_for_indra')

        return [x.get_sparql() for x in result][:top_k]

    def get_domains_of_property(self, property):
        result = self.graph.query(self.domains_query, initBindings={"prop": URIRef(property)})
        return [x[0].toPython() for x in result]

_qp = QueryProposalManager()




def get_query_proposal(text, classes=None, top_k=5):
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
    return _qp.get_query_proposals_from_plaintext(text, classes, top_k)
