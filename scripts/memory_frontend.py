#!/usr/bin/env python
import roslib
import rospy
from tracking.msg import Tracking

roslib.load_manifest('mario')

# To read the log file for Information about tracking
from rosgraph_msgs.msg import Log

pub = rospy.Publisher('/tracker_wrapper/tracked_persons', Tracking, queue_size=10)


def process_log(data):
    """
    Processing the log message to undertake certain actions.

    If the message is sent by 'openni_tracker' node it's interesting.

    If message starts with "Calibration Complete" it means that the
    tracker is actually tracking a person. Extract the number of the person.
    (and save it in some way)

    If message starts with "Lost User". it means the tracker lost a person.
    Extract the number and process it further.
    :type data: rosgraphs_msgs.msg.Log
    """
    if data.name == "/openni_tracker":
        message = data.msg
        if message.startswith("Calibration c"):
            number = int(message.rsplit(None, 1)[-1])
            print("calibration complete for user %d" % number)
            pub.publish(Tracking(True, number))
        if message.startswith("Lost"):
            number = int(message.rsplit(None, 1)[-1])
            print("lost user %d" % number)
            pub.publish(Tracking(False, number))


def main():
    rospy.init_node("tracker_wrapper")
    rospy.Subscriber("/rosout", Log, process_log)

    rospy.spin()


if __name__ == '__main__':
    main()
