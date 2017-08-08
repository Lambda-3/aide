from aide_messages.msg import ShortMessage
from rospy import loginfo
from aide_core.extractors import AbstractTopicExtractor
from aide_core.namespaces import whatsapp


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/aide/incoming_whatsapp_messages"
    type = ShortMessage
    queue_size = 43
    rate = 2

    def extract(self):
        msg = self.message
        sender = msg.other
        content = msg.content
        loginfo(msg)
        subj = whatsapp._
        subj.a = "WhatsAppMessage"
        subj.sender = sender
        subj.content = content
        loginfo(subj)
        return subj
