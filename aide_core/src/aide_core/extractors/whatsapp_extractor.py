from aide_messages.msg import ShortMessage
from aide_core.extractors import AbstractTopicExtractor
from aide_core.namespaces import whatsapp
from aide_core.apis import natural_language


class WhatsAppExtractor(AbstractTopicExtractor):
    from_channel = "/aide/incoming_whatsapp_messages"
    type = ShortMessage
    queue_size = 43
    rate = 2

    def extract_from_message(self, message):
        sender = message.other
        content = message.content

        subj = whatsapp._
        subj.a = "WhatsAppMessage"
        subj.sender = sender
        subj.content = content
        subj.intent = natural_language.get_intent(content)

        return subj
