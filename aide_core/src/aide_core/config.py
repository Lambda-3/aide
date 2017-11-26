from os.path import abspath

PROJECT_PATH = abspath(__file__ + "../../../../")
SOURCE_PATH = abspath(PROJECT_PATH + "/src/aide_core/")
APIS_PATH = abspath(SOURCE_PATH + "/apis/")
ACTION_PROVIDERS_PATH = abspath(SOURCE_PATH + "/actions/")
EXTRACTORS_PATH = abspath(SOURCE_PATH + "/extractors/")
NAMESPACES_FORMAT = "turtle"
NAMESPACES_PATH = abspath(PROJECT_PATH + "/Namespaces.rdf")
