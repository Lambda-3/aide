import rospy
from std_msgs.msg import Bool

__pub = rospy.Publisher("/aide/ac_control", Bool, queue_size=42)


def shutdown():
    """
    Shuts down the air conditioning system.
    """
    pass


def increase_performance(room):
    """
    Increases the performance of the AC adaptor in a given room by one step.


    :param room: ID of the room
    :type room: str
    :return:
    """
    __pub.publish(Bool(True))


def decrease_performance(room):
    """
    Decreases the performance of the AC adaptor in a given room by one step.


    :param room: ID of the room
    :type room: str
    :return:
    """
    __pub.publish(Bool(False))


def set_performance(room, percentage):
    """
    Sets the performance of the AC adaptor in a given room to a given percentage.

    :param room: ID of the room
    :type room: str
    :param percentage: Percentage of the AC performance, integer (0 - 100).
    :type percentage: int
    """
    pass