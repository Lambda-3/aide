#!/usr/bin/env python
import roslib
import tf


roslib.load_manifest('mario')

import rospy
import threading
from mario.msg import Skeleton
from mario.srv import TrackSkeleton
from geometry_msgs.msg import Point

# To read the log file for Information about tracking
from rosgraph_msgs.msg import Log

# hack at time being
if __name__ == "__main__":
    # Set of tracked IDs
    tracked_ids = set()
    rospy.init_node("tracker_wrapper")
    listener = tf.TransformListener()
    # lock to synchronize access to tracked IDs.
    tracked_lock = threading.Lock()


class LogHandler:
    def __init__(self, log_channel_name, srv_handler):
        """

        :type out: rospy.Publisher
        :type srv_handler: ServiceHandler
        """
        self._srv_handler = srv_handler
        rospy.Subscriber(log_channel_name, Log, self.processLog)

    def processLog(self, data):
        """
        Processing the log message to undertake certain actions.

        If the message is sent by 'openni_tracker' node it's interesting.

        If message starts with "Calibration Complete" it means that the
        tracker is actually tracking a person. Extract the number of the
        person.
        (and save it in some way)

        If message starts with "Lost User". it means the tracker lost a person.
        Extract the number and process it further.
        """
        if (data.name == "/openni_tracker"):
            message = data.msg
            if (message.startswith("Calibration c")):
                number = int(message.rsplit(None, 1)[-1])
                print("calibration complete for user %d" % number)

                with tracked_lock:
                    tracked_ids.add(number)
                    self._srv_handler.call_service(True, number)

            if (message.startswith("Lost")):
                number = int(message.rsplit(None, 1)[-1])
                print("lost user %d" % number)

                with tracked_lock:
                    try:
                        tracked_ids.remove(number)
                        self._srv_handler.call_service(False, number)
                    except KeyError:
                        print("User %d was not tracked." % number)


class ServiceHandler:
    SERVICE_CHANNEL = 'mario/skeleton_rec/track_skeleton'

    def __init__(self, listener):
        self.listener = listener

    @staticmethod
    def get_service(callback):
        return rospy.Service(ServiceHandler.SERVICE_CHANNEL, TrackSkeleton,
                             callback)

    @staticmethod
    def call_service(is_tracked, id):
        rospy.wait_for_service(ServiceHandler.SERVICE_CHANNEL)
        try:
            track_skeleton = rospy.ServiceProxy(ServiceHandler.SERVICE_CHANNEL,
                                                TrackSkeleton)
            track_skeleton(is_tracked, Skeleton(id, get_head_position(id)))
        except rospy.ServiceException:
            pass


def get_head_position(id):
    rospy.sleep(5)
    if id in tracked_ids:
        (trans, rot) = listener.lookupTransform('/head_%d' % id,
                                                '/openni_depth_frame',
                                                rospy.Time(0))
        return Point(*trans)
    return Point(0, 0, 0)


def main():
    srv_handler = ServiceHandler(tf.TransformListener())

    log_handler = LogHandler("/rosout", srv_handler)

    rospy.spin()


if __name__ == '__main__':
    main()
