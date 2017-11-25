"""
Sample action provider implementation for a fictious room.

Used for the evaluation of the system.
"""

import rospy
from std_msgs.msg import String

__pub = rospy.Publisher("/aide/building_control", String, queue_size=42)


def deactivate_power():
    """
    Shuts down the power supply in the building. Does not affect the power supply of the lights.
    """
    __pub.publish(String("Power deactivated!"))


def activate_power():
    """
    Activates the power supply in the building.
    """
    pass


def activate_sprinkler(room):
    """
    Activates the anti-fire sprinklers in a given room.
    :param room:
    type
    :return:
    """
    __pub.publish(String("Sprinklers activated in room {}".format(room)))


def switch_lighting_mode(mode):
    """
    Switches the lighting mode of the building to a given mode.

    Possible mode values are:
        - normal: normal lighting mode
        - night: minimal lighting mode (dimmed lights, shorter lighting periods)
        - emergency: minimal lighting mode, emergency exit signs extra bright
        - off: light switched off.

    :param mode: Mode to switch to.
    :type mode: str

    Defaults to normal.
    """
    mode = mode if mode in ("night", "emergency", "off") else "normal"
    __pub.publish("Switched lighting mode to: {}".format(mode))


def broadcast_message(message):
    """
    Broadcasts a message in the whole building.
    :param message: Message to broadcast.
    :type message: str
    """

    __pub.publish(String("Broadcasting message: {}".format(message)))
