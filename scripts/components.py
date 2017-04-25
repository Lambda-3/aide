import re
import logging
from abc import ABCMeta, abstractmethod

module_logger = logging.getLogger("mario.components")


class FunctionHandler:
    __metaclass__ = ABCMeta

    def __init__(self, function_store):
        self.function_store = function_store
        self.logger = logging.getLogger("FunctionHandler")

    @abstractmethod
    def add_function(self):
        pass

    @staticmethod
    def build_from_text(function_text):
        name = re.findall("def (\w+)", function_text)[0]
        module_logger.info("Building function with name {}".format(name))
        # !!! IN NO WAY IS THIS SECURE !!!
        exec (function_text)
        return locals()[name]


class FunctionStorage:
    __metaclass__ = ABCMeta

    @abstractmethod
    def store_function(self, name, doc, function_text):
        pass

    @abstractmethod
    def load_function(self, name, doc, function_text):
        pass

    @abstractmethod
    def get_all_stored(self):
        pass


class KnowledgeBase:
    __metaclass__ = ABCMeta


class Environment:
    __metaclass__ = ABCMeta

    def __init__(self, knowledge_base):
        pass

    @abstractmethod
    def spin(self):
        pass
