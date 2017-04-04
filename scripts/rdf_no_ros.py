#!/usr/bin/env python2

import logging
import time

from rdf_constants import PATH, LOGPATH
from rest import RestApi
from rules import RuleHandler

logger = logging.getLogger("mario")
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler(LOGPATH)
fh.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - %(name)s %(levelname)s - %("
                              "message)s")
fh.setFormatter(formatter)
ch.setFormatter(formatter)
logger.addHandler(fh)
logger.addHandler(ch)


class FakeApi:
    def __getattr__(self, name):
        def dummy_func(*args, **kwargs):
            print("Fake-Invoking function {}:".format(name))
            if args:
                print("Arguments: " + str(args))
            if kwargs:
                print("Keyword Arguments: " + str(kwargs))

        self.__dict__[name] = dummy_func

        return dummy_func


def main():
    logger.info("Creating a RuleHandler with a FakeApi.")
    with RuleHandler(PATH, FakeApi()) as graph:
        logger.info("Parsing graph...")
        graph.parse("simpleOnthology.rdf", format="turtle")
        logger.info("graph parsed.")
        graph.add_cleanup_function(lambda: graph.remove((None, None, None)))

        api = RestApi(graph)
        api.run()
        running = True
        while running:
            try:
                graph.execute_rules()
                # graph.pprint()
                time.sleep(3)
            except KeyboardInterrupt:
                logger.info("Shutting down!")
                running = False


if __name__ == "__main__":
    main()
