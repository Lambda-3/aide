#!/usr/bin/env python
import numpy
import roslib

roslib.load_manifest('mario')
import rospy
import threading
import tf
from mario.msg import Tracking
from mario.srv import AddSkeleton, DelSkeleton, SetDistance

# To read the log file for Information about tracking
from rosgraph_msgs.msg import Log

# Set of tracked IDs
tracked_ids = set()

# lock to synchronize access to tracked IDs.
tracked_lock = threading.Lock()


class LogHandler:
    def __init__(self, log_channel_name, out, srv_handler):
        self._out = out
        self._srv_handler = srv_handler
        rospy.Subscriber(log_channel_name, Log, self.processLog)

    def processLog(self, data):
        '''
        Processing the log message to undertake certain actions.
        
        If the message is sent by 'openni_tracker' node it's interesting.
        
        If message starts with "Calibration Complete" it means that the
        tracker is actually tracking a person. Extract the number of the
        person.
        (and save it in some way)
        
        If message starts with "Lost User". it means the tracker lost a person.
        Extract the number and process it further.
        '''
        if (data.name == "/openni_tracker"):
            message = data.msg
            if (message.startswith("Calibration c")):
                number = int(message.rsplit(None, 1)[-1])
                print("calibration complete for user %d" % number)
                self._out.publish(Tracking(True, number))

                with tracked_lock:
                    tracked_ids.add(number)
                    self._srv_handler.add_skeleton(number)

            if (message.startswith("Lost")):
                number = int(message.rsplit(None, 1)[-1])
                print("lost user %d" % number)
                self._out.publish(Tracking(False, number))

                with tracked_lock:
                    tracked_ids.remove(number)
                    self._srv_handler.del_skeleton(number)


class DistanceHandler:
    def __init__(self, srv_handler):
        self._listener = tf.TransformListener()
        self._srv_handler = srv_handler

    def process_tracked_ids(self):
        with tracked_lock:
            for i in tracked_ids:
                trans = self.get_distance(i)
                dist = numpy.linalg.norm(trans)
                self._srv_handler.set_distance(i, dist)
                print(dist)

    def get_distance(self, id):
        try:
            (trans, rot) = self._listener.lookupTransform(
                '/torso_%s' % str(id),
                '/openni_depth_frame',
                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            print e

        return trans

    def run(self, rate):
        while not rospy.is_shutdown():
            self.process_tracked_ids()
            rate.sleep()


class ServiceHandler:
    def add_skeleton(self, id):
        rospy.wait_for_service("add_skeleton")
        try:
            add_skeleton = rospy.ServiceProxy("add_skeleton", AddSkeleton)
            add_skeleton(id)
        except rospy.ServiceException as e:
            pass

    def del_skeleton(self, id):
        rospy.wait_for_service("add_skeleton")
        try:
            del_skeleton = rospy.ServiceProxy("del_skeleton", DelSkeleton)
            del_skeleton(id)
        except rospy.ServiceException as e:
            pass

    def set_distance(self, id, distance):
        rospy.wait_for_service("set_distance")
        try:
            set_distance = rospy.ServiceProxy("set_distance", SetDistance)
            set_distance(id, distance)
        except rospy.ServiceException as e:
            pass


def main():
    rospy.init_node("tracker_wrapper")
    pub = rospy.Publisher('/tracker_wrapper/tracked_persons', Tracking,
                          queue_size=10)

    srv_handler = ServiceHandler()

    log_handler = LogHandler("/rosout", pub, srv_handler)

    distance_handler = threading.Thread(target=
                                        DistanceHandler(srv_handler).run,
                                        args=(rospy.Rate(10),))
    distance_handler.start()

    rospy.spin()


if __name__ == '__main__':
    main()
