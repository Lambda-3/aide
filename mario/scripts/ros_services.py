import re
import rospy
import mario_messages.srv as services

from rospy import loginfo, logdebug

from rospy_message_converter.message_converter import convert_dictionary_to_ros_message as dtr

service_handlers = {}

first_cap_re = re.compile('(.)([A-Z][a-z]+)')
all_cap_re = re.compile('([a-z0-9])([A-Z])')


def cc_to_underscore(name):
    s1 = first_cap_re.sub(r'\1_\2', name)
    return all_cap_re.sub(r'\1_\2', s1).lower()


def get_service_handler(name):
    """

    :type name: str, object
    :returns ServiceHandler
    """
    if not isinstance(name, str):
        name = name.__name__
    try:
        return service_handlers[name]
    except KeyError:
        raise ValueError("Service with name {} is not implemented!".format(name))


class ServiceHandler:
    """
    Offers the registration and access to an implemented ros service.
    """

    def __init__(self, service_name):
        # import the auto generated class corresponding to the service
        exec """from mario_messages.srv import {}""".format(service_name)
        # service channel is the service name converted to underscore notation
        self.SERVICE_CHANNEL = "mario/{}".format(cc_to_underscore(service_name))
        self.service_name = service_name
        self.service_class = eval(service_name)

    def register_service_raw(self, callback):
        """
        Registers the service with a callback function.
        
        :param callback: callback function to be executed when the service is called. 
        :return: service object.
        """

        return rospy.Service(self.SERVICE_CHANNEL, self.service_class, callback)

    def register_service(self, callback):
        """
        
        Registers the service with a callback function while wrapping the arguments and the return of the callback function into 
        a response class object.
        
        This way, dicts also can be used as return types.
        
        :return: service object.
        """

        def improved_callback(req):
            # unwrap the request, but only the top level
            result = callback(**{x: req.__getattribute__(x) for x in req.__class__.__slots__})
            response_class = self.service_class._response_class
            slot_types = response_class._slot_types
            slot_names = response_class.__slots__

            if result is None:
                # return None if service response is not supposed to be empty, if it's supposed to be empty,
                # return an empty tuple (first one shows up as error, second one doesn't)
                return None if slot_types else ()

            response = {}
            number_of_slots = len(slot_types)

            if number_of_slots == 1:
                result = (result,)

            for i in range(number_of_slots):
                result_in_slot = result[slot_names[i]] if isinstance(result, dict) else result[i]
                logdebug("Result in slot: {}".format(result_in_slot))
                slot_name = slot_names[i]
                slot_type = slot_types[i]

                # this means slot is supposed to be a list
                if "[]" in slot_type:
                    if not (isinstance(result_in_slot, list) or isinstance(result_in_slot, tuple)):
                        result_in_slot = (result_in_slot,)

                    if not result_in_slot:
                        response[slot_name] = []
                        continue

                    # this means the list entries are dicts
                    if isinstance(result_in_slot[0], dict):
                        response_slot = []
                        for entry in result_in_slot:
                            # transform every dict to a message
                            response_slot.append(dtr(slot_type[:-2], entry))
                        response[slot_name] = response_slot
                    # this means the list entries are native
                    elif "/" not in slot_type or (
                            # this means the list entries are messages
                                isinstance(result_in_slot[0],
                                           services.genpy.Message) and result_in_slot[0]._type == slot_type[:-2]):
                        response[slot_name] = result_in_slot
                    else:
                        raise ValueError("{} is a list but neither a dict nor a Ros Message!".format(result_in_slot))



                # this means the slot is supposed to be a ros native data type
                elif "/" not in slot_type or (
                        # this means the result is already a message
                            isinstance(result_in_slot, services.genpy.Message) and result_in_slot._type == slot_type):
                    logdebug("Slot is instance of {}".format(slot_type))
                    response[slot_name] = result_in_slot


                elif isinstance(result_in_slot, dict):
                    response[slot_name] = dtr(self.service_class._response_class._slot_types[i], result_in_slot)
                    logdebug("Slot is a dict")

                else:
                    raise ValueError("{} is neither a dict nor a Ros Message!".format(result_in_slot))
            loginfo("Returning: {} of type {}".format(response, type(response)))
            return response

        return rospy.Service(self.SERVICE_CHANNEL, self.service_class, improved_callback)

    def get_service(self):
        """
        Returns the service which then can be called.
        
        !! Does not call the service.
        
        :return: the service. 
        """
        rospy.wait_for_service(self.SERVICE_CHANNEL)
        return rospy.ServiceProxy(self.SERVICE_CHANNEL, self.service_class)

    def call_service(self, **kwargs):
        rospy.wait_for_service(self.SERVICE_CHANNEL)
        return rospy.ServiceProxy(self.SERVICE_CHANNEL, self.service_class)(**kwargs)


# Create a list of all Services that are implemented
implemented_services = [k[1:] for k in services.__dict__.keys() if k.startswith("_") and not k.startswith("__")]

# create a service handler instance for each implemented service
for service_name in implemented_services:
    service_handler = ServiceHandler(service_name)
    service_handlers[service_name] = service_handler
