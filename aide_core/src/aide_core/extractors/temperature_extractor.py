from aide_core.extractors import AbstractTopicExtractor
from aide_messages.msg import AirConditionerMessage
from aide_core.namespaces import ac_control


class TemperatureExtractor(AbstractTopicExtractor):
    from_channel = "/aide/temperature"
    type = AirConditionerMessage
    queue_size = 43
    rate = 1

    def extract_from_message(self, message):
        """

        :type message: AirConditionerMessage
        """
        subj = ac_control._
        subj.room = message.id
        subj.temperature = message.temperature
        

        return subj
