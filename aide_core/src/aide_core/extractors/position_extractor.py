import rospy
from tf.listener import TransformListener

from aide_core.extractors import AbstractPeriodicExtractor
from aide_core.namespaces import robot


class PositionExtractor(AbstractPeriodicExtractor):
    MAP_LINK = "/map"
    ROBOT_LINK = "/base_footprint"
    rate = 1
    queue_size = 42

    def __init__(self, **kwargs):
        super(PositionExtractor, self).__init__(**kwargs)
        self.listener = TransformListener()

    def extract(self):
        """
        Hurr durr.
        
        """
        try:
            ((x, y, _), _) = self.listener.lookupTransform(self.MAP_LINK, self.ROBOT_LINK, rospy.Time(0))
        except:
            return None

        subj = robot.self
        subj.position_x = x
        subj.position_y = y
        return subj
