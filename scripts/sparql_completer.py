import json
import re

import requests
from rdflib.graph import ConjunctiveGraph
from rdflib.plugins.sparql.processor import prepareQuery

property_path_depth_2 = """
SELECT DISTINCT ?prop ?range ?instances ?prop2 ?range2 ?instances2 WHERE {
    ?prop rdfs:domain ?aClass.
    ?prop rdfs:range ?range.
        OPTIONAL{
            ?instances a ?range.
            }
        OPTIONAL {
            ?prop2 rdfs:domain ?range.
            ?prop2 rdfs:range ?range2.
            OPTIONAL {
                ?instances2 a ?range2.
                }
            }
    }
"""

property_path_depth_1 = """
SELECT DISTINCT ?prop ?range ?instances WHERE {
    ?prop rdfs:domain ?aClass.
    ?prop rdfs:range ?range.
        OPTIONAL{
            ?instances a ?range.
            }
    }
"""

subject_and_class_query = """
SELECT DISTINCT ?instance ?class WHERE {
    ?instance a ?class.
    }
"""

first_cap_re = re.compile('(.)([A-Z][a-z]+)')
all_cap_re = re.compile('([a-z0-9])([A-Z])')


def cc_to_space(name):
    """
    Converts camel-case to spaces.
    
    :param name: 
    :return: 
    """
    s1 = first_cap_re.sub(r'\1 \2', name)
    return all_cap_re.sub(r'\1 \2', s1).lower()


class QueryProposalManager:
    def __init__(self):
        self.objects = []
        self.ctr = 0

    def append(self, qp):
        self.objects.append(qp)
        qp.seq_nr = self.ctr
        self.ctr = self.ctr + 1
        return self.ctr

    def get(self, seq_nr):
        return self.objects[seq_nr]

    def __getitem__(self, item):
        return self.objects[item]


class QueryProposal:
    def __init__(self, row):
        self.row = row
        # Gets rid of all optional unset values, converts URIs to SPARQL notation.
        self.list = [":".join(x.toPython().split("/")[-2:]) for x in row if x]

    def get_for_indra(self):
        """
        Creates a indra-suitable representation of the query proposal path.
        
        That is, gets rid of all namespaces, classes and xsd, concatenates remaining words to one string while 
        separating the camel-cased ones.
        
        :return: Indra-suitable representation of the query proposal.
        """
        return " ".join(cc_to_space(x.split(":")[1]) for x in self.list if not x.startswith("classes") and not
        "XMLSchema#" in x)

    def __eq__(self, other):
        return self.list == other.list

    def __str__(self):
        return str(self.list)

    def __repr__(self):
        return repr(self.list)

    def get_sparql(self):
        """
        Creates a sparql query proposal.
        
        1. if last Term is an xsd (contains XML), turn it into ?lit
        2. If last term is a class, turn it into ?ref
        3. if last Term is neither a class nor xsd, delete the next to last entry (which is a class then).
        4. Turn every classes into [ BNodes.
        5. for number of classes turned into BNodes, close all BNodes at the end.
        
        :return: valid sparql query proposal.
        """
        ret_list = list(self.list)
        size = len(ret_list)
        # 1
        if "XMLSchema#" in ret_list[-1]:
            ret_list[-1] = "?lit"
        # 2
        elif "classes:" in ret_list[-1]:
            ret_list[-1] = "?ref"
        # 3
        elif "classes:" in ret_list[-2]:
            ret_list.pop(size - 2)
        else:
            raise ValueError("Something somewhere went terribly wrong...")
        # 4
        size = len(ret_list)
        bnodes = 0
        for i in range(size):
            if "classes:" in ret_list[i]:
                ret_list[i] = "["
                bnodes = bnodes + 1
        # 5
        ret_list += ["]" for x in range(bnodes)]
        return " ".join(ret_list) + " ."

    def get_path(self):
        ret_list = list(self.list)
        for i in range(len(ret_list)):
            if ret_list[i].startswith("properties"):
                # property path symbol
                ret_list[i] = "--" + ret_list[i] + "-->"
            if "XMLSchema#" in ret_list[i]:
                # transfer all xmlschema into xsd:
                ret_list[i] = "xsd:" + ret_list[i].split("#")[1]

        if not ret_list[-1].startswith("classes:") and not ret_list[-1].startswith("xsd:"):
            ret_list[-1] = ". " + ret_list[-1]
            ret_list.append(" a {}".format(ret_list[-2]))

        return "".join(ret_list)


class QueryCompleter:
    def __init__(self, onthology_graph_path, kb_graph_path, format_onthology="n3", format_kb="n3"):
        self.onthology_graph = ConjunctiveGraph()
        self.onthology_graph.parse(onthology_graph_path, format=format_onthology)
        self.kb_graph = ConjunctiveGraph()
        self.kb_graph.parse(kb_graph_path, format=format_kb)
        self.property_path_depth_1 = prepareQuery(property_path_depth_1)
        self.property_path_depth_2 = prepareQuery(property_path_depth_2)
        self.subject_and_class_query = prepareQuery(subject_and_class_query)

    def get_subject_from_plaintext(self, subject):
        result = self.kb_graph.query(self.subject_and_class_query)
        return [{"instance": ":".join(row[0].toPython().split("/")[-2:]),
                 "class"   : row[1].toPython().split("/")[-1]} for row in result]

    def get_indra_results(self, pairs):
        data = {'corpus'       : 'wiki-2014',
                'model'        : 'W2V',
                'language'     : 'EN',
                'scoreFunction': 'COSINE', 'pairs': pairs}

        headers = {
            'content-type': "application/json"
        }

        response = requests.request("POST", "http://localhost:8916/relatedness", data=json.dumps(data), headers=headers)
        response.raise_for_status()
        return response.json()['pairs']

    def get_queries_from_plaintext_and_subject(self, subject_class, plain_text, top_k):
        query_proposals = QueryProposalManager()

        result = self.onthology_graph.query(self.property_path_depth_2, initBindings={"aClass": subject_class})
        for row in result:
            qp = QueryProposal(row)
            query_proposals.append(qp) if qp not in query_proposals else None

        result = self.onthology_graph.query(self.property_path_depth_1, initBindings={"aClass": subject_class})
        for row in result:
            qp = QueryProposal(row)
            query_proposals.append(qp) if qp not in query_proposals else None

        # enumerate to later be able to get by index. indra skips numbers anyways
        pairs = [{"t1": plain_text, "t2": "{}. {}".format(qp.seq_nr, qp.get_for_indra())} for qp in query_proposals]
        response = self.get_indra_results(pairs)

        # get the top k matches from the response, get their sparql representation and return them as a list
        return [{"code": query_proposals[(int(rsp['t2'].split(".")[0]))].get_sparql(),
                 "path": query_proposals[(int(rsp['t2'].split(".")[0]))].get_path()}
                for rsp in sorted(response, reverse=True)[:top_k]]
