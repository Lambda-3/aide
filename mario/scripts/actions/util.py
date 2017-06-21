from actions import *
def type_of_argument(arg):
    print(type(arg))


def dispatch_command(command):
    """

    :type command: str
    """
    (provider, action) = command.split(".", 1)
    print(command)
    eval(command)