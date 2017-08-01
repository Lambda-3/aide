from uuid import uuid4 as uuid

from aide_core.apis.rdf_utils import Graph, Triple, robot, properties
from std_msgs.msg import String

from aide_core.extractors import AbstractTopicExtractor


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/aide/console_input"
    type = String
    queue_size = 43
    rate = 2

    def __init__(self, **kwargs):
        super(InputExtractor, self).__init__(**kwargs)

    def extract(self):
        split_args = self.message.data.split(" ")
        command = split_args[0]
        command_id = robot[uuid().hex]
        if len(split_args) == 1:
            return Graph(Triple(command_id, properties.command, command))
        args = split_args[1:]
        return Graph(Triple(command_id, properties.command, command),
                     *(Triple(command_id, properties["arg{}".format(i)], arg) for i, arg in enumerate(args)))
