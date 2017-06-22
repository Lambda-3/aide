from rospy import loginfo

from actions import *
from apis import *

from mario_messages.msg._RdfGraphStamped import RdfGraphStamped
import rospy

from apis.rdf_utils import Triple, Graph

__pub = rospy.Publisher("/mario/rdf", RdfGraphStamped, queue_size=42)


def type_of_argument(arg):
    print(type(arg))

def print_arg(arg):
    print(arg)

def dispatch_command(command):
    """

    :type command: str
    """
    loginfo("Executing {} of type {}".format(command, type(command)))
    rslt = eval(command)
    loginfo("Got result: {}".format(rslt))
    return None


def add_to_stream(subject, predicate, object):
    __pub.publish(Graph(Triple(subject, predicate, object)))
