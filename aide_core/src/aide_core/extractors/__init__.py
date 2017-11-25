import threading
from abc import ABCMeta, abstractmethod, abstractproperty

import genpy
import rospy
from rospy import loginfo, logwarn
from rospy.core import logerror
from rospy.exceptions import ROSSerializationException

from aide_core.apis.rdf_utils import Graph


class AbstractExtractor(object):
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        loginfo("Creating class {}...".format(self.__class__.__name__))
        self.publisher = kwargs['publisher']
        self.lock = threading.Lock()
        self.register_me()
        self.to_reimport = []
        self.initialize()

    def initialize(self):
        pass

    def get_time(self):
        return rospy.Time().now()

    def register_me(self):
        loginfo("Registering thread...")
        thread = threading.Thread(target=self.run_forever)
        self._finished = False
        thread.daemon = True
        thread.start()

    def publish(self, message):
        if message:
            if type(message) == list or type(message) == tuple:
                graph = Graph(*message)
            else:
                graph = Graph(message)
            if graph:
                try:
                    self.publisher.publish(graph)
                except ROSSerializationException as e:
                    logerror(str(e))
            else:
                logwarn("{} extracted something that is not publishable!".format(self.__class__.__name__))

    def run_forever(self):
        loginfo(self.finished)
        while not self.finished:
            try:
                result = self.loop()
                self.publish(result)
            except Exception as e:
                logwarn("ERROR: {}".format(type(e)))


    @abstractmethod
    def loop(self):
        pass

    @property
    def finished(self):
        return self._finished or rospy.is_shutdown()

    def finish(self):
        loginfo("{} finished.".format(self.__class__.__name__))
        self._finished = True


class AbstractPeriodicExtractor(AbstractExtractor):
    def __init__(self, **kwargs):
        super(AbstractPeriodicExtractor, self).__init__(**kwargs)

    @abstractmethod
    def extract(self):
        pass

    @abstractproperty
    def queue_size(self):
        pass

    @abstractproperty
    def rate(self):
        pass

    def loop(self):
        loginfo("getting result")
        result = self.extract()
        loginfo("got result")
        rospy.Rate(self.rate).sleep()
        return result
        # rospy.sleep(self.rate)


class AbstractTopicExtractor(AbstractPeriodicExtractor):
    def __init__(self, **kwargs):
        super(AbstractTopicExtractor, self).__init__(**kwargs)
        self.new = True

        def set_msg(data):
            self._msg = data
            self.new = True

        self.subscriber = rospy.Subscriber(self.from_channel, self.type, set_msg)

    @property
    def queue_size(self):
        return 42

    @abstractproperty
    def from_channel(self):
        """

        :rtype: str 
        """
        pass

    @property
    def only_new(self):
        """

        :rtype: str 
        """
        return True

    @abstractproperty
    def type(self):
        """

        :rtype: genpy.Message
        """
        pass
    @abstractmethod
    def extract_from_message(self, message):
        pass

    def extract(self):
        return self.extract_from_message(self.message)

    def loop(self):
        if getattr(self, 'new', False):
            result = self.extract()
            if result:
                self.publish(result)
            self.new = not self.only_new
        rospy.Rate(self.rate).sleep()

    @property
    def message(self):
        try:
            return self._msg
        except AttributeError:
            name = self.__class__.__name__
            loginfo("{} is waiting for the first message...".format(name))
            while "_msg" not in self.__dict__ and not rospy.is_shutdown():
                rospy.Rate(self.rate).sleep()
            if not rospy.is_shutdown():
                loginfo("{} got first message.".format(name))
                return self._msg
