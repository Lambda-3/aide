import rospy
import tf
from mario_messages.msg import RdfGraphStamped, RdfTripleStamped
from rospy import loginfo
from tf2_msgs.msg._TFMessage import TFMessage

from apis.rdf_utils import literalize, Graph, Triple
from extractors import AbstractExtractor


class PositionExtractor(AbstractExtractor):
    MAP_LINK = "/map"
    ROBOT_LINK = "/base_footprint"
    rate = 1
    queue_size = 42

    def __init__(self, **kwargs):
        super(PositionExtractor, self).__init__(**kwargs)
        self.listener = tf.TransformListener()

    def extract(self):
        """

        :type message: TFMessage
        """
        time = self.get_time()
        try:
            ((x, y, z), rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            return Graph(Triple("mario:self", "properties:position_x", x, time),
                         Triple("mario:self", "properties:position_y", y, time))
        except:
            return None
