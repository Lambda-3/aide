from os.path import abspath

PROJECT_PATH = abspath(__file__ + "../../../")
SCRIPTS_PATH = abspath(PROJECT_PATH + "/scripts")
NAMESPACES_PATH = abspath(SCRIPTS_PATH + "/Namespaces.rdf")
NAMESPACES_FORMAT = "turtle"
ONTHOLOGY_PATH = abspath(SCRIPTS_PATH + "/simpleOnthology.rdf")
LOGGING_PATH = abspath(PROJECT_PATH + "/log/mario.log")
RDF_PATH = abspath(PROJECT_PATH + "/.rdf/mario")
