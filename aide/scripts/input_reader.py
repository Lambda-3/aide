#!/usr/bin/env python
import atexit
import code
import os

import rospy
import roslib
import readline
from rospy import loginfo
from std_msgs.msg import String

import config

roslib.load_manifest("aide")


class HistoryConsole(code.InteractiveConsole):
    def __init__(self, locals=None, filename="<console>",
                 histfile=os.path.expanduser(config.PROJECT_PATH + "/.input-hist")):
        code.InteractiveConsole.__init__(self, locals, filename)
        self.init_history(histfile)

    def init_history(self, histfile):
        readline.parse_and_bind("tab: complete")
        if hasattr(readline, "read_history_file"):
            try:
                readline.read_history_file(histfile)
            except IOError:
                pass
            atexit.register(self.save_history, histfile)

    def save_history(self, histfile):
        readline.set_history_length(1000)
        readline.write_history_file(histfile)


def main():
    rospy.init_node("input_reader")
    pub = rospy.Publisher("/aide/console_input", String, queue_size=42)
    console = HistoryConsole()
    console.preprocess = lambda source: source[7:]
    atexit.register(loginfo, "Going down by user-input.")
    while not rospy.is_shutdown():
        try:
            command = console.raw_input("aide> ")
            pub.publish(command)
        except EOFError:
            print("")
            rospy.signal_shutdown("Going down.")


if __name__ == '__main__':
    main()
