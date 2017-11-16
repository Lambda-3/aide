from aide_core.namespaces import robot
from std_msgs.msg import String
from aide_core.extractors import AbstractTopicExtractor


class InputExtractor(AbstractTopicExtractor):
    from_channel = "/aide/console_input"
    type = String
    queue_size = 43
    rate = 2

    def extract_from_message(self, message):
        split_args = message.data.split(" ")

        command = split_args[0]
        args = split_args[1:]
        
        subj = robot._
        subj.command = command
        for i, arg in enumerate(args):
            subj["argument%d" % i] = arg

        return subj
