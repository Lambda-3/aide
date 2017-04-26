import re
import rospy
import mario.srv as services

service_handlers = {}

first_cap_re = re.compile('(.)([A-Z][a-z]+)')
all_cap_re = re.compile('([a-z0-9])([A-Z])')


def cc_to_underscore(name):
    s1 = first_cap_re.sub(r'\1_\2', name)
    return all_cap_re.sub(r'\1_\2', s1).lower()


def get_service_handler(name):
    """

    :type name: str
    :returns ServiceHandler
    """
    return service_handlers[name]


class ServiceHandler:
    """
    Offers the registration and access to an implemented ros service.
    """

    def __init__(self, service_name):
        # import the auto generated class corresponding to the service
        exec """from mario.srv import {}""".format(service_name)
        # service channel is the service name converted to underscore notation
        self.SERVICE_CHANNEL = "mario/{}".format(cc_to_underscore(service_name))
        self.service_name = service_name
        self.service_class = eval(service_name)

    def register_service(self, callback):
        """
        Registers the service with a callback function.
        
        :param callback: callback function to be executed when the service is called. 
        :return: service object.
        """
        return rospy.Service(self.SERVICE_CHANNEL, self.service_class, callback)

    def get_service(self):
        """
        Returns the service which then can be called.
        
        !! Does not call the service.
        :return: the service. 
        """
        rospy.wait_for_service(self.SERVICE_CHANNEL)
        return rospy.ServiceProxy(self.SERVICE_CHANNEL, self.service_class)


# Create a list of all Services that are implemented
implemented_services = [k[1:] for k in services.__dict__.keys() if k.startswith("_") and not k.startswith("__")]

# create a service handler instance for each implemented service
for service_name in implemented_services:
    service_handler = ServiceHandler(service_name)
    service_handlers[service_name] = service_handler
