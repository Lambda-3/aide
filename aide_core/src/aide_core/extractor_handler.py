#!/usr/bin/env python
import importlib
import os
import re
import traceback

import imp
import reimport
import roslib
import rospy
from aide_messages.msg import RdfGraphStamped
from aide_messages.srv import GetAllExtractors, AddExtractor
from aide_messages.srv._GetExtractor import GetExtractor
from rospy import loginfo
from rospy.core import logerror
from std_msgs.msg import Bool, String

from aide_core import config
from aide_core.apis import util
from apis.ros_services import cc_to_underscore, get_service_handler

roslib.load_manifest("aide_core")

excluded = ["speech_extractor.py"]


class ExtractorHandler(object):
    def __init__(self):
        self.load_all_extractors()
        self.extractors = []
        extractor_files = self.get_all_extractor_files()

        for file_name in extractor_files:
            loginfo("Working on: {}".format(file_name))
            with open(file_name, "r") as f:
                file_content = f.read()
            self.add_extractor(f.name.rsplit("/", 1)[-1], file_content, loading=True)

    def load_all_extractors(self):
        pass

    def reload_api_references(self):
        for module in reimport.modified(config.APIS_PATH):
            if not module == "__main__" and not module == "__mp_main__":
                loginfo("Module {} changed. reloading...".format(module))
                reimport.reimport(module)

    def get_extractor(self, name):
        try:
            with open("{}/{}.py".format(config.EXTRACTORS_PATH, name), "r") as f:
                file_content = f.read()
            return file_content
        except IOError as e:
            if "No such file or directory" in e:
                loginfo(e.message)
                return None
            else:
                raise IOError(e)

    def register_extractor(self, ExtractorClass):
        """

        :type ExtractorClass: AbstractExtractor
        """
        # name = ExtractorClass.__name__
        publisher = rospy.Publisher("/aide/rdf", RdfGraphStamped, queue_size=ExtractorClass.queue_size)

        extractor = ExtractorClass(publisher=publisher)

        # subscriber = rospy.Subscriber(extractor.from_channel, extractor.type, callback)

        self.extractors.append(extractor)

    def add_extractor(self, name, file_content, loading=False):
        class_name = re.findall("class\s+(.*?)[\(,:]", file_content)[0]
        loginfo("Class name: {}".format(class_name))
        if name.endswith(".py"):
            name = name[:-3]
        extractor_name = name
        file_name = extractor_name + ".py"
        loginfo("File name: {}".format(file_name))

        loginfo("   Compiling...")
        try:
            compile(file_content, file_name, "exec")
        except Exception as e:
            return (False, traceback.format_exc(limit=0))

        loginfo("   ...compiled!")
        with open(config.EXTRACTORS_PATH + "/" + file_name, "w+") as f:
            f.write(file_content)

        lint_output = ""
        if not loading:
            loginfo("   applying Pylint...")
            lint_output = util.apply_lint(config.EXTRACTORS_PATH + "/" + file_name)

        loginfo("   Importing...")
        try:
            # exec ("""import extractors.{0} as {0}""".format(extractor_name))
            extractor_module = importlib.import_module("aide_core.extractors.{}".format(extractor_name))
            loginfo(extractor_module)
            imp.reload(extractor_module)

            ExtractorClass = extractor_module.__dict__[class_name]
        except Exception as e:
            logerror(e)
            return False, lint_output + traceback.format_exc(1)
        loginfo("   ...imported!")
        self.register_extractor(ExtractorClass)
        loginfo("   ...registered!")
        return True, lint_output

    def get_all_extractors(self):
        return [api_path.rsplit("/", 1)[-1].rsplit(".py", 1)[-2] for api_path in self.get_all_extractor_files()]

    def get_all_extractor_files(self):
        """

        :rtype: generator
        """
        api_files = []
        for (dirpath, _, file_names) in os.walk(config.EXTRACTORS_PATH):
            file_names = [f for f in file_names if f not in excluded]
            api_files.extend([dirpath + "/" + x for x in file_names if not x.startswith("__") and x.endswith(".py")])
        return api_files


if __name__ == '__main__':
    rospy.init_node("extractor")

    publisher = rospy.Publisher("/aide/test_node", String, queue_size=50)

    loginfo("Creating extractor handler...")
    extractor_handler = ExtractorHandler()
    loginfo("Registering services...")

    rospy.Subscriber("/aide/update_apis", Bool, lambda x: extractor_handler.reload_api_references() if x.data else ())

    get_service_handler(GetExtractor).register_service(lambda name: extractor_handler.get_extractor(name) or ())
    get_service_handler(GetAllExtractors).register_service(extractor_handler.get_all_extractors)
    get_service_handler(AddExtractor).register_service(extractor_handler.add_extractor)

    loginfo("Registered services. Spinning.")

    rospy.spin()
