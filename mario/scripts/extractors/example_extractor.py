import rospy
from mario_messages.msg import RdfGraphStamped, RdfTripleStamped
from extractors import AbstractTopicExtractor
from apis import simple as sa

from std_msgs.msg import String


class ExampleExtractor(AbstractTopicExtractor):
    from_channel = "/mario/test_node"
    type = String
    queue_size = 42
    rate = 1

    @property
    def msg(self):
        return None

    def __init__(self, **kwargs):
        super(ExampleExtractor, self).__init__(**kwargs)
        # self.msg = None

    def extract(self):
        return RdfGraphStamped([RdfTripleStamped("mario:c", "mario:d", sa.fancify_string(self.message.data),
                                                 self.get_time())])
