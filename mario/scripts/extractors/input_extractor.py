from rospy import loginfo

from apis.rdf_utils import Graph, Triple, mario, properties
from extractors import AbstractTopicExtractor
from uuid import uuid4 as uuid
from std_msgs.msg import String


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/mario/console_input"
    type = String
    queue_size = 42
    rate = 2

    def __init__(self, **kwargs):
        super(InputExtractor, self).__init__(**kwargs)

    def extract(self):
        split_args = self.message.data.split(" ")
        command = split_args[0]
        command_id = mario[uuid().hex]
        if len(split_args) == 1:
            return Graph(Triple(command_id, properties.command, command))
        args = split_args[1]
        if len(split_args) == 2: args = [args]
        return Graph(Triple(command_id, properties.command, command),
                     *(Triple(command_id, properties["arg{}".format(i)], arg) for i, arg in enumerate(args)))
