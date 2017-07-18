#!/usr/bin/env python
import roslib
import rospy
from std_msgs.msg import Bool

roslib.load_manifest("mario")

from mario_messages.srv._GetEvent import GetEvent
from mario_messages.srv._GetRoutine import GetRoutine

from apis import storage
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as rtd

import json
import threading
import mario_messages.msg
from mario_messages.msg import FiredEvent, Event
from mario_messages.srv import CallFunction, AddEvent, AddRule, GetAllRoutines, GetAllEvents
from rospy import loginfo
from rospy.core import logwarn
from apis.ros_services import get_service_handler, get_slots

call = get_service_handler(CallFunction).get_service()
register_event = get_service_handler(AddEvent).get_service()


class Step(object):
    pass


class Execute(Step):
    def __init__(self, function, mapping=dict(), literals=dict()):
        self.what = function
        self.mapping = mapping
        # self.parametrized = True if mapping else False
        self.literals = literals

    def fill_parameters(self, params):
        """

        :type params: dict
        """
        for argument_name, parameter_name in self.mapping.iteritems():
            self.literals[argument_name] = params[parameter_name]
        for name, value in self.literals.iteritems():
            loginfo("sanitizing {}: {}".format(name, value))
            try:
                self.literals[name] = value.format(**params)
            except AttributeError:
                # if it's not string, just don't do anything
                pass

    def execute(self):
        loginfo("Executing function {} with arguments {}".format(self.what, self.literals))
        call(self.what, json.dumps(self.literals))


class WaitFor(Step):
    def __init__(self, event_name):
        self.event_name = event_name


class RepeatUntil(Step):
    def __init__(self, wait_for, execute_steps, rate=1):
        self.execute_steps = execute_steps
        self.event_name = wait_for
        self.rate = rospy.Rate(rate)


class Routine(object):
    def __eq__(self, other):
        return self.name == other.name

    def __init__(self, name, trigger_name, execution_steps, event_notifier, description=""):
        self.name = name
        self.description = description
        self.trigger_name = trigger_name
        self.execution_steps = self.parse_steps(execution_steps)
        self.running = False
        self.waiting_for = None
        self.waiting_for_lock = threading.Lock()
        self.event_notifier = event_notifier

    def parse_steps(self, steps):
        """

        """
        result = []
        for step in steps:
            loginfo("Parsing step {}".format(step))
            if isinstance(step, Step):
                result.append(Step)
            else:
                try:
                    dct = step
                    Class = eval(dct['type'])
                except TypeError:
                    dct = json.loads(step)
                    Class = eval(dct['type'])

                result.append(Class(**{k: v for k, v in dct.iteritems() if not k == 'type'}))
        loginfo("Execution steps: {}".format(result))
        return result

    def check_triggered_by(self, event):
        """

        :type event: Event
        """
        if self.trigger_name == event.name:
            loginfo("Event {} triggered routine {}".format(event.name, self.name))
            self.params = event.params
            threading.Thread(target=self.trigger).start()
            # self.trigger()

    def trigger(self):
        loginfo("Routine {} triggered.".format(self.name))
        self.running = True
        for step in self.execution_steps:
            if (isinstance(step, Execute)):
                step.fill_parameters(self.params)
                step.execute()

            if (isinstance(step, WaitFor)):
                self.waiting_for = step.event_name
                loginfo("Waiting for event {}...".format(step.event_name, str(self.params)))
                self.event_notifier.add_to_waiting(self, self.waiting_for)
                while self.waiting_for:
                    rospy.Rate(2).sleep()
            if (isinstance(step, RepeatUntil)):
                self.waiting_for = step.event_name
                loginfo("Waiting for event {}...".format(step.event_name, str(self.params)))
                self.event_notifier.add_to_waiting(self, self.waiting_for)
                rate = step.rate
                while self.waiting_for:
                    for execute_step in step.execute_steps:
                        execute_step.execute()
                        rate.sleep()

        self.running = False
        loginfo("Routine {} is done.".format(self.name))

    def check_waiting_for(self, event):
        with self.waiting_for_lock:
            if self.waiting_for == event.name:
                loginfo("Routine is waiting for event {}".format(event.name))
                if all(val == self.params[key] for key, val in event.params.iteritems() if key in self.params.keys()):
                    self.params.update(event.params)
                    self.waiting_for = None
                    self.event_notifier.remove_from_waiting(self, event.name)
                    loginfo("{} is done waiting.".format(self.name))
                else:
                    loginfo("Event parameters didn't match.")


class WaitingRoutinesHandler(object):
    def __init__(self):
        self.waiting_routines_lock = dict()
        self.waiting_routines = dict()

    def get_lock(self, event_name):
        """
        
        :param event_name: 
        :type event_name: str
        :return: 
        :rtype: threading.Lock
        """
        try:
            return self.waiting_routines_lock[event_name]
        except KeyError:
            self.waiting_routines_lock[event_name] = threading.Lock()
            return self.waiting_routines_lock[event_name]

    def add_to_waiting(self, routine, event_name):
        with self.get_lock(event_name):
            loginfo("Appending {} to waiting routines".format(routine.name))
            try:
                self.waiting_routines[event_name].append(routine)
            except KeyError:
                self.waiting_routines[event_name] = [routine]

    def remove_from_waiting(self, routine, event_name):
        with self.get_lock(event_name):
            loginfo("Removing {} from waiting routines.".format(routine.name))
            try:
                self.waiting_routines[event_name].remove(routine)
            except ValueError:
                logwarn("Trying to remove a routine from an event consumer, but the routine was not waiting for it.")

    def handle_waiting_routines(self, event):
        """

        :type event: FiredEvent
        """
        try:
            waiting_routines = self.waiting_routines[event.name]
            loginfo("{} routine(s) are waiting for this event.".format(len(waiting_routines)))
        except KeyError:
            return
        for routine in waiting_routines:
            loginfo("Checking if event {} is what {} waiting for...".format(event.name, routine.name))
            routine.check_waiting_for(event)


class EventHandler(object):
    def __init__(self):
        self.event_notifier = WaitingRoutinesHandler()
        self.event_storage = storage.get_collection("events")
        self.routine_storage = storage.get_collection("routines")
        self.routines = []
        self.load_routines()
        self.load_events()
        storage.get_collection(name="routines", indices=["name"])
        storage.get_collection(name="events", indices=["name"])

    def load_routines(self):
        # TODO
        loginfo("Loading routines...")
        # self.routines = [Routine("test_routine", "test", self.event_notifier,
        #                          [Execute("util.print_arg", literals={"arg": 3}),
        #                           RepeatUntil("test2", [Execute("util.print_arg", literals={"arg": 4})]
        #                                       )
        #                           ])
        #                  ]
        loaded_routines = self.routine_storage.find(select_only=get_slots(mario_messages.msg.Routine))
        loginfo("...loaded routines!")
        loginfo("Appending Routines...")
        for routine in loaded_routines:
            loginfo("Appending routine {}".format(routine['name']))
            self.routines.append(Routine(event_notifier=self.event_notifier, **routine))
        loginfo("...done!")

    def load_events(self):
        loginfo("Loading events...")
        loaded_events = self.event_storage.find(select_only=get_slots(Event))
        for event in loaded_events:
            loginfo("Loading event {}".format(event))
            register_event(Event(**event))

    def get_event(self, name):
        loginfo("Getting event {}".format(name))
        return self.event_storage.find_one({"name": name})

    def get_routine(self, name):
        return self.routine_storage.find_one({"name": name})

    def get_all_routines(self):
        return self.routine_storage.find({})

    def get_all_events(self):
        return self.event_storage.find({})

    def add_rule(self, routine, events):
        """

        :type routine: mario_messages.msg.Routine
        """
        self.add_routine(routine)
        for event in events:
            self.add_event(event)
        return True

    def add_event(self, event):
        """

        :type event: mario_messages.msg.Event
        """
        self.event_storage.insert(entry=event)
        register_event(event)

    def add_routine(self, routine):
        """

        :type routine: mario_messages.msg.Routine
        """
        decoded_steps = [json.loads(step) for step in routine.execution_steps]
        #routine.execution_steps = decoded_steps
        loginfo("Adding routine {}..".format(routine.name))
        result = self.routine_storage.find_one(where={"name": routine.name})
        parsed_routine = Routine(event_notifier=self.event_notifier, **rtd(routine))

        if result:
            loginfo("Routine with this name already exists...")
            self.routines.remove(parsed_routine)
            self.routine_storage.replace_one(replace_with=routine, where={"name": routine.name})
        else:
            self.routine_storage.insert(entry=routine)
        self.routines.append(parsed_routine)
        loginfo("Routine added.")
        return True

    def handle_incoming(self, event):
        """

        :type event: mario_messages.msg.FiredEvent
        """
        loginfo("Incoming event: {}".format(event.name))
        event.params = json.loads(event.params)
        for routine in self.routines:
            routine.check_triggered_by(event)

        self.event_notifier.handle_waiting_routines(event)


if __name__ == '__main__':
    rospy.init_node("event_handler")

    loginfo("Creating event handler...")
    event_handler = EventHandler()
    # rospy.Subscriber("/mario/update_apis", Bool, lambda x: _reload_api_references() if x.data else ())
    rospy.Subscriber("/mario/events", FiredEvent, event_handler.handle_incoming)

    loginfo("Registering services...")
    get_service_handler(AddRule).register_service(event_handler.add_rule)
    get_service_handler(GetAllRoutines).register_service(event_handler.get_all_routines)
    get_service_handler(GetRoutine).register_service(lambda name: event_handler.get_routine(name))
    get_service_handler(GetEvent).register_service(lambda name: event_handler.get_event(name) or ())

    get_service_handler(GetAllEvents).register_service(event_handler.get_all_events)
    loginfo("Registered services. Spinning.")

    rospy.spin()
