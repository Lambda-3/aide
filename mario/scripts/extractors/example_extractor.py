from mario_messages.msg import RdfGraphStamped, RdfTripleStamped
from extractors import AbstractExtractor
from apis import simple as sa

from std_msgs.msg import String


class ExampleExtractor(AbstractExtractor):
    from_channel = "/mario/test_node"
    type = String
    queue_size = 42

    def transform(self, message):
        return RdfGraphStamped([RdfTripleStamped("mario:c", "mario:d", sa.fancify_string(message.data),
                                                 self.get_time())])
