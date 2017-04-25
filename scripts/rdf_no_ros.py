#!/usr/bin/env python2

import logging
import time
import re

from components import Environment
from config import RDF_PATH, LOGGING_PATH, ONTHOLOGY_PATH
from rest import RestApi
from rules import RuleHandler

logger = logging.getLogger("mario")
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(LOGGING_PATH)
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter("%(asctime)s - %(name)s %(levelname)s - %("
                              "message)s")
fh.setFormatter(formatter)
ch.setFormatter(formatter)
logger.addHandler(fh)
logger.addHandler(ch)


class FakeApi:
    def __init__(self):
        self.logger = logging.getLogger("mario.rdf_no_ros.FakeApi")

    def __getattr__(self, name):
        if not self.__dict__[name]:
            def dummy_func(*args, **kwargs):
                logger.info("Fake-Invoking function {}:".format(name))
                if args:
                    logger.info("Arguments: " + str(args))
                if kwargs:
                    logger.info("Keyword Arguments: " + str(kwargs))

            self.__dict__[name] = dummy_func

        return self.__dict__[name]

    def add_function(self, funct):
        logger.info("Adding function with name {}".format(funct.__name__))
        self.__dict__[funct.__name__] = funct.__get__(self, type(self))
        return funct.__name__

    @staticmethod
    def build_function(function_text):
        name = re.findall("def (\w+)", function_text)[0]
        logger.info("Building function with name {}".format(name))
        # !!! IN NO WAY IS THIS SECURE !!!
        exec (function_text)
        return locals()[name]


class NoRosEnvironment(Environment):
    def spin(self):
        logger.info("Creating a RuleHandler with a FakeApi.")
        with RuleHandler(RDF_PATH, FakeApi(), ONTHOLOGY_PATH) as graph:
            graph.add_cleanup_function(lambda: graph.remove((None, None, None)))

            api = RestApi(graph)
            api.run()
            running = True
            while running:
                try:
                    graph.execute_rules()
                    graph.pprint()
                    time.sleep(3)
                except KeyboardInterrupt:
                    logger.info("Shutting down!")
                    running = False


if __name__ == "__main__":
    NoRosEnvironment().spin()
