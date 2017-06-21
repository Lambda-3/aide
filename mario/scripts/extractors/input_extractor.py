from apis.rdf_utils import Graph, Triple, mario, properties
from extractors import AbstractTopicExtractor
from uuid import uuid4 as uuid
from std_msgs.msg import String


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/mario/console_input"
    type = String
    queue_size = 42
    rate = 2

    def extract(self):
        return Graph(Triple(mario[uuid().hex], properties.command, self.message.data))
