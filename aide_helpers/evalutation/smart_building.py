#!/usr/bin/env python
import atexit
import json
from code import InteractiveConsole as Console

import roslib
import rospy
from rospy import loginfo
from threading import Thread
from std_msgs.msg import  String

roslib.load_manifest("aide_helpers")


class SmartBuildingSimulator(object):
    def __init__(self):
        self.temperature = 23.0
        self.on_fire = False
        self.persons = 10
        self.fire_department_arrived = False
        self.ended = False

    def set_on_fire(self, room):
        loginfo("Fire detected in room {}!".format(room))
        self.on_fire = True
        self.room_nr = room

    def handle_incoming(self, msg):
        loginfo(msg)

    def start_simulation(self):
        loginfo("Beginning simulation")
        rate = rospy.Rate(1)
        while not self.on_fire:
            rate.sleep()

        loginfo("{} persons in building!".format(self.persons))
        while not self.persons <= 0:
            self.persons -= 1
            loginfo("{} persons in building!".format(self.persons))
            rate.sleep()

        loginfo("Building is empty!")

        while not self.fire_department_arrived:
            rate.sleep()
        loginfo("Fire department has arrived!")
        rate.sleep()
        self.on_fire = False
        loginfo("Simulation ended.")

    def get_state(self):
        return {
            "rooms": [
                {
                    "roomNr": 1,
                    "fireAlarmTriggered": True if self.on_fire and self.room_nr == 1 else False,
                    "roomAlias": "WC"

                },
                {
                    "roomNr": 2,
                    "fireAlarmTriggered": True if self.on_fire and self.room_nr == 2 else False,
                    "roomAlias": "bureau"
                },
                {
                    "roomNr": 3,
                    "fireAlarmTriggered": True if self.on_fire and self.room_nr == 3 else False,
                    "roomAlias": "lobby"
                },
            ],
            "doors": [
                {
                    "state": "open"
                },
                {
                    "state": "closed"
                },
                {
                    "state": "open"
                },
                {
                    "state": "locked"
                },
            ],
            "controlOverride": self.fire_department_arrived,
            "people": self.persons,
            "isEmpty": self.persons == 0
        }


def publish(building):
    pub = rospy.Publisher("/aide/building", String, queue_size=42)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown() and not building.ended:
        pub.publish(json.dumps(building.get_state()))
        rate.sleep()


def main():
    rospy.init_node("evaluation_building")
    building = SmartBuildingSimulator()
    rospy.Subscriber("/aide/building_control", String, callback=lambda msg: building.handle_incoming(msg.data))

    console = Console()
    console.preprocess = lambda source: source[3:]
    atexit.register(loginfo, "Going down by user-input.")
    building_thread = Thread(target=building.start_simulation)
    building_thread.daemon = True
    pub_thread = Thread(target=publish, args=(building,))
    pub_thread.daemon = True
    pub_thread.start()
    while not rospy.is_shutdown():
        try:
            command = console.raw_input("sb> ")
            if command == "start":
                building_thread.start()
            if command.startswith("fire"):
                room = command.split(" ")[1]
                building.set_on_fire(int(room))
            if command == "rescue":
                building.fire_department_arrived = True
            if command == "end":
                return

        except EOFError:
            print("")
            building.finished = True
            rospy.signal_shutdown("Going down.")


if __name__ == '__main__':
    main()
