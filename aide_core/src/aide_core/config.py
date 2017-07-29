from os.path import abspath

PROJECT_PATH = abspath(__file__ + "../../../")
SOURCE_PATH = abspath(PROJECT_PATH + "/src/")
APIS_PATH = abspath(SOURCE_PATH + "/apis/")
ACTION_PROVIDERS_PATH = abspath(SOURCE_PATH + "/actions/")
EXTRACTORS_PATH = abspath(SOURCE_PATH + "/extractors/")
NAMESPACES_PATH = abspath(SOURCE_PATH + "/Namespaces.rdf")
NAMESPACES_FORMAT = "turtle"
ONTHOLOGY_PATH = abspath(SOURCE_PATH + "/simpleOnthology.rdf")
LOGGING_PATH = abspath(PROJECT_PATH + "/log/aide.log")
RDF_PATH = abspath(PROJECT_PATH + "/.rdf/aide")
