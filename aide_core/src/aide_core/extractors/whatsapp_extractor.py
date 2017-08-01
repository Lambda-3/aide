from uuid import uuid4 as uuid

from aide_core.apis.rdf_utils import Graph, Triple, robot, properties, rdf, classes
from aide_messages.msg import ShortMessage

from aide_core.extractors import AbstractTopicExtractor


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/aide/incoming_whatsapp_messages"
    type = ShortMessage
    queue_size = 43
    rate = 2

    def __init__(self, **kwargs):
        super(InputExtractor, self).__init__(**kwargs)

    def extract(self):
        msg = self.message
        sender = msg.other
        id = robot[uuid().hex]
        content = msg.content

        return Graph(Triple(id, rdf.type, classes.WhatsAppMesage),
                     Triple(id, properties.sender, sender),
                     Triple(id, properties.content, content))
