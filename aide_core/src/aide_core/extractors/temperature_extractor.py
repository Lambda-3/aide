from aide_core.extractors import AbstractTopicExtractor
from aide_messages.msg import AirConditionerMessage
from aide_core.namespaces import roomControl
class TemperatureExtractor(AbstractTopicExtractor):
    from_channel = "/aide/temperature"
    type = AirConditionerMessage
    queue_size = 43
    rate = 2

    def extract_from_message(self, message):
        """

        :type message: AirConditionerMessage
        """
        subj = roomControl.conditioner
        subj.temperature = message.temperature
        subj.roomID = message.id




        return subj
