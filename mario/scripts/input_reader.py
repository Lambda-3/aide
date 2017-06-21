#!/usr/bin/env python
import rospy
import roslib
from rospy import loginfo
from std_msgs.msg import String

roslib.load_manifest("mario")


def main():
    rospy.init_node("input_reader")
    pub = rospy.Publisher("/mario/console_input", String, queue_size=42)
    while not rospy.is_shutdown():
        try:
            command = raw_input("mario> ")
            pub.publish(command)
        except EOFError:
            print("\n")
            loginfo("Going down.")
            rospy.signal_shutdown("Going down.")

if __name__ == '__main__':
    main()
