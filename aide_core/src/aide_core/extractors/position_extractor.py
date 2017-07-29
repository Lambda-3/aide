import rospy
from apis.rdf_utils import Graph, Triple, robot, properties
from tf.listener import TransformListener
from tf2_msgs.msg._TFMessage import TFMessage

from aide_core.extractors import AbstractExtractor


class PositionExtractor(AbstractExtractor):
    MAP_LINK = "/map"
    ROBOT_LINK = "/base_footprint"
    rate = 1
    queue_size = 42

    def __init__(self, **kwargs):
        super(PositionExtractor, self).__init__(**kwargs)
        self.listener = TransformListener()

    def extract(self):
        """

        :type message: TFMessage
        """
        time = self.get_time()
        try:
            ((x, y, z), rot) = self.listener.lookupTransform(self.MAP_LINK, self.ROBOT_LINK, rospy.Time(0))
            return Graph(Triple(robot.self, properties.position_x, x, time),
                         Triple(robot.self, properties.position_y, y, time))
        except:
            return None
