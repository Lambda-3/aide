import rospy
from aide_messages.msg import ShortMessage
from aide_core.apis.rdf_utils import Triple, Graph
from rospy import loginfo

__pub = rospy.Publisher("/aide/outgoing_whatsapp_messages", ShortMessage, queue_size=42)

def send_message_to_me(content):
    """
    
    :param content: message content.
    :type content: str
    """
    message = ShortMessage(type="WhatsApp", content=content)
    __pub.publish(message)

def type_of_argument(arg):
    """
    
    :param arg: this is some dope argument.
    """
    
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


def test_function(number, string, boolean):
    """
    Just a test function.
    
    :type number: int
    :type string: str
    :type boolean: bool
    """
    loginfo("Yaaay. number {} of {}. string {} of {}. boolean {} of {}".format(
        number, type(number), string, type(string), boolean, type(boolean)))


def add_to_stream(subject, predicate, object):
    __pub.publish(Graph(Triple(subject, predicate, object)))
