#!/usr/bin/env python
import atexit
from code import InteractiveConsole as Console

from rospy import Rate
import roslib
import rospy
from rospy import loginfo
from threading import Thread
from std_msgs.msg import Bool

from aide_messages.msg import AirConditionerMessage

roslib.load_manifest("aide_helpers")


class ACControllerSimulator(object):
    def __init__(self):
        self.temperature = 23.0
        self.increased = False
        self.decreased = False
        self.finished = False

    def increase_ac(self):
        loginfo("Got signal to increase AC!")
        self.increased = True

    def decrease_ac(self):
        loginfo("Got signal to decrease AC!")
        self.decreased = True

    def simulate(self):
        loginfo("Beginning simulation")
        rate = Rate(2)
        self.temperature = 23.0
        while not self.increased and not self.finished:
            self.temperature += 0.1
            rate.sleep()
            loginfo(self.temperature)
        while not self.decreased and not self.finished:
            self.temperature -= 0.1
            rate.sleep()
            loginfo(self.temperature)

        while self.temperature < 22.5 and not self.finished:
            self.temperature += 0.1

        loginfo("Simulation ended.")

def publish(ac):
    pub = rospy.Publisher("/aide/temperature", AirConditionerMessage, queue_size=42)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub.publish(AirConditionerMessage(ac.temperature, "simulatedRoom0", rospy.get_rostime()))
        rate.sleep()

def main():
    rospy.init_node("evaluation_ac")
    ac = ACControllerSimulator()
    rospy.Subscriber("/aide/ac_control", Bool, callback=lambda msg: ac.increase_ac() if msg.data else ac.decrease_ac())

    console = Console()
    console.preprocess = lambda source: source[3:]
    atexit.register(loginfo, "Going down by user-input.")
    ac_thread = Thread(target=ac.simulate)
    ac_thread.daemon = True
    pub_thread = Thread(target=publish, args=(ac, ))
    pub_thread.daemon = True
    pub_thread.start()
    while not rospy.is_shutdown():
        try:
            command = console.raw_input("ac> ")
            if command == "start":
                ac_thread.start()
            if command == "end":
                return

        except EOFError:
            print("")
            ac.finished = True
            rospy.signal_shutdown("Going down.")


if __name__ == '__main__':
    main()
