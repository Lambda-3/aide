#!/usr/bin/env python
import importlib
import json
import threading

import aide_messages
import reimport
import roslib
import rospy
from aide_messages.msg import FiredEvent, EventListener
from aide_messages.srv import (CallFunction, AddEventListener, RemoveEventListener, AddRule, GetAllRoutines,
                               GetAllEventListeners, GetRoutine, GetEventListener)
from aide_messages.srv._DeleteRoutine import DeleteRoutine
from rospy import loginfo
from rospy.core import logwarn
from std_msgs.msg import String

import apis
from apis import storage
from apis.ros_services import get_service_handler, get_slots
from rospy_message_converter.message_converter import convert_ros_message_to_dictionary as rtd

roslib.load_manifest("aide_core")
call = get_service_handler(CallFunction).get_service()
register_event_listener = get_service_handler(AddEventListener).get_service()
unregister_event_listener = get_service_handler(RemoveEventListener).get_service()


def parse_steps(steps):
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

            result.append(Class(**{k: v for k, v in dct.items() if not k == 'type'}))
    loginfo("Execution steps: {}".format(result))
    return result


class Step(object):
    def __repr__(self):
        return "{class_name}({attributes})".format(
            class_name=self.__class__.__name__,
            attributes=", ".join("{}={}".format(name, value) for name, value in self.__dict__.items())
        )


class Execute(Step):
    def __init__(self, function, mapping=None, literals=None, funcs=None):
        self.what = function
        self.mapping = mapping or dict()
        self.literals = literals or dict()
        self.funcs = funcs or dict()

    def fill_parameters(self, params):
        """

        :type params: dict
        """
        # insert parameters mappings as arguments
        for argument_name, parameter_name in self.mapping.items():
            self.literals[argument_name] = params[parameter_name]
        # evaluate functions as arguments
        for argument, func in self.funcs.items():
            api, name = func['function'].split(".", 2)
            actual_func = importlib.import_module("apis.{}".format(api, func)).__dict__[name]

            for arg_name, param_name in func['mapping'].items():
                func['literals'][arg_name] = params[param_name]

            self.literals[argument] = actual_func(**func['literals'])

    def execute(self):
        loginfo("Executing function {} with arguments {}".format(self.what, self.literals))

        call(self.what, json.dumps(self.literals))


class WaitFor(Step):
    def __init__(self, event_name):
        self.event_name = event_name


class RepeatUntil(Step):
    def __init__(self, wait_for, execute_steps, rate=1):
        if not isinstance(execute_steps, list):
            execute_steps = [execute_steps]
        self.execute_steps = parse_steps(execute_steps)
        self.event_name = wait_for
        self.rate = rospy.Rate(rate)


class Routine(object):
    def __eq__(self, other):
        return self.name == other.name

    def __init__(self, name, trigger_name, execution_steps, event_notifier, description=""):
        self.name = name
        self.description = description
        self.trigger_name = trigger_name
        self.execution_steps = parse_steps(execution_steps)
        self.running = False
        self.waiting_for = None
        self.waiting_event = None
        self.waiting_for_lock = threading.Lock()
        self.event_notifier = event_notifier
        self.shutdown = False

    def unregister(self):
        self.shutdown = True
        with self.waiting_for_lock:
            if self.waiting_for != None:
                self.waiting_event.set()

    def check_triggered_by(self, event):
        """

        :type event: Event
        """
        if self.trigger_name == event.name and not self.running:
            loginfo("Event {} triggered routine {}".format(event.name, self.name))
            self.params = event.params
            threading.Thread(target=self.trigger).start()


    def trigger(self):
        loginfo("Routine {} triggered.".format(self.name))
        self.running = True
        for step in self.execution_steps:
            if self.shutdown:
                self.running = False
                return

            if (isinstance(step, Execute)):
                step.fill_parameters(self.params)
                step.execute()

            if (isinstance(step, WaitFor)):
                self.waiting_for = step.event_name
                loginfo("Waiting for event {}...".format(step.event_name))
                self.event_notifier.add_to_waiting(self, self.waiting_for)
                self.waiting_event = threading.Event()

                while not self.waiting_event.is_set():
                    self.waiting_event.wait()

                self.event_notifier.remove_from_waiting(self, self.waiting_for)
                self.waiting_for = None
                loginfo("{} is done waiting.".format(self.name))
                self.waiting_event = None

            if (isinstance(step, RepeatUntil)):
                self.waiting_for = step.event_name
                loginfo("Waiting for event {}...".format(step.event_name, str(self.params)))
                self.event_notifier.add_to_waiting(self, self.waiting_for)
                self.waiting_event = threading.Event()

                while not self.waiting_event.is_set():
                    for execute_step in step.execute_steps:
                        execute_step.execute()
                        step.rate.sleep()

                self.event_notifier.remove_from_waiting(self, self.waiting_for)
                self.waiting_for = None
                loginfo("{} is done waiting.".format(self.name))
                self.waiting_event = None

        self.running = False
        loginfo("Routine {} is done.".format(self.name))

    def check_waiting_for(self, event):
        with self.waiting_for_lock:
            if self.waiting_for == event.name:
                loginfo("Routine is waiting for event {}".format(event.name))
                # if params match, s.t. all params in event that are present in this routine are equal
                if all(val == self.params[key] for key, val in event.params.items() if key in self.params.keys()):
                    self.params.update(event.params)
                    self.waiting_event.set()
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
            loginfo("Appended. Now it looks like this: {}".format(self.waiting_routines[event_name]))

    def remove_from_waiting(self, routine, event_name):
        with self.get_lock(event_name):
            loginfo("Removing {} from routines waiting for {}.".format(routine.name, event_name))
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
        self.event_listeners_storage = storage.get_collection("event_listeners")
        self.routine_storage = storage.get_collection("routines")
        self.routines = []
        self.load_routines()
        self.load_event_listeners()
        storage.get_collection(name="routines", indices=["name"])
        storage.get_collection(name="events", indices=["name"])

    def reload_api_references(self, name):
        loginfo("Reloading {}".format(name))
        try:
            reimport.reimport('aide_core.apis.{}'.format(name))
        except ValueError:
            apis._update()
            reimport.reimport('aide_core.apis.{}'.format(name))

    def load_routines(self):
        loginfo("Loading routines...")
        loaded_routines = self.routine_storage.find(select_only=get_slots(aide_messages.msg.Routine))
        loginfo("...loaded routines!")
        loginfo("Appending Routines...")
        for routine in loaded_routines:
            loginfo("Appending routine {}".format(routine['name']))
            self.routines.append(Routine(event_notifier=self.event_notifier, **routine))
        loginfo("...done!")

    def load_event_listeners(self):
        loginfo("Loading events...")
        loaded_events = self.event_listeners_storage.find(select_only=get_slots(EventListener))
        for event_listener in loaded_events:
            loginfo("Loading event {}".format(event_listener))
            register_event_listener(EventListener(**event_listener))

    def get_event_listener(self, name):
        loginfo("Getting event {}".format(name))
        result = self.event_listeners_storage.find_one(where={"name": name},
                                                       select_only=get_slots(aide_messages.msg.EventListener))
        return result or {}

    def get_all_event_listeners(self):
        return self.event_listeners_storage.find({})

    def get_routine(self, name):
        return self.routine_storage.find_one(where={"name": name}, select_only=get_slots(aide_messages.msg.Routine))

    def delete_routine(self, name):
        result = self.routine_storage.find_one(where={"name": name}, select_only=get_slots(aide_messages.msg.Routine))
        # routine exists
        if result:
            loginfo("Deleting routine {}".format(name))
            idx = self.routines.index(Routine(event_notifier=self.event_notifier, **result))
            routine = self.routines.pop(idx)
            routine.unregister()
            self.routine_storage.delete_one(where={"name": name})
            return True
        else:
            return False

    def get_all_routines(self):
        return self.routine_storage.find({})

    def add_rule(self, routine, event_listeners):
        """

        :type routine: aide_messages.msg.Routine
        """

        if not routine.name:
            return False, "Routine needs a name!"
        faulty_event_listener = None
        for event_listener in event_listeners:
            if not faulty_event_listener:
                success = self.add_event_listener(event_listener)
                if not success:
                    loginfo("Got error trying to register events. undoing changes.")
                    faulty_event_listener = event_listener

        if faulty_event_listener:
            for event_listener in event_listeners:
                self.remove_event_listener(event_listener.name)
            return False, "Event listener {} has an illegal definition.. Does it have a name? Is the SPARQL Clause valid?".format(
                faulty_event_listener.name)
        else:
            self.add_routine(routine)
            return True, None

    def remove_event_listener(self, name):
        self.event_listeners_storage.delete_one(where={"name": name})
        try:
            unregister_event_listener(name)
        except:
            loginfo("Got error unregistering event.")
            return False

        return True

    def add_event_listener(self, event):
        """

        :type event: aide_messages.msg.EventListener
        """

        # add where and enclose in braces
        event.sparqlWhere = "WHERE {{ {} }}".format(event.sparqlWhere)
        result = self.event_listeners_storage.find_one(where={"name": event.name})
        if result:
            loginfo("Event with this name already exists...")
            self.event_listeners_storage.replace_one(replace_with=event, where={"name": event.name})
        else:
            self.event_listeners_storage.insert(entry=event)

        try:
            register_event_listener(event)
        except:
            loginfo("Got error registering event.")
            return False

        return True

    def add_routine(self, routine):
        """

        :type routine: aide_messages.msg.Routine
        """
        decoded_steps = [json.loads(step) for step in routine.execution_steps]
        # routine.execution_steps = decoded_steps
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

        :type event: aide_messages.msg.FiredEvent
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
    rospy.Subscriber("/aide/events", FiredEvent, event_handler.handle_incoming)

    rospy.Subscriber("/aide/update_apis", String, lambda x: event_handler.reload_api_references(x.data))

    loginfo("Registering services...")
    get_service_handler(AddRule).register_service(event_handler.add_rule)
    get_service_handler(GetAllRoutines).register_service(event_handler.get_all_routines)
    get_service_handler(GetRoutine).register_service(lambda name: event_handler.get_routine(name))
    get_service_handler(GetEventListener).register_service(lambda name: event_handler.get_event_listener(name) or ())
    get_service_handler(DeleteRoutine).register_service(event_handler.delete_routine)

    get_service_handler(GetAllEventListeners).register_service(event_handler.get_all_event_listeners)
    loginfo("Registered services. Spinning.")

    rospy.spin()
