import rospy
from aide_messages.msg._RdfGraphStamped import RdfGraphStamped
from apis import *
from apis.rdf_utils import Triple, Graph
from rospy import loginfo

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


def print_all_args(**kwargs):
    loginfo("Printing all args")
    for (k, v) in kwargs.items():
        loginfo("\t name:{} - type: {} - value: {}".format(k, type(v), v))


def add_to_stream(subject, predicate, object):
    __pub.publish(Graph(Triple(subject, predicate, object)))
