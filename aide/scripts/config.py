from os.path import abspath

PROJECT_PATH = abspath(__file__ + "../../../")
SCRIPTS_PATH = abspath(PROJECT_PATH + "/scripts/")
APIS_PATH = abspath(SCRIPTS_PATH + "/apis/")
ACTION_PROVIDERS_PATH = abspath(SCRIPTS_PATH + "/actions/")
EXTRACTORS_PATH = abspath(SCRIPTS_PATH + "/extractors/")
NAMESPACES_PATH = abspath(SCRIPTS_PATH + "/Namespaces.rdf")
NAMESPACES_FORMAT = "turtle"
ONTHOLOGY_PATH = abspath(SCRIPTS_PATH + "/simpleOnthology.rdf")
LOGGING_PATH = abspath(PROJECT_PATH + "/log/aide.log")
RDF_PATH = abspath(PROJECT_PATH + "/.rdf/aide")
