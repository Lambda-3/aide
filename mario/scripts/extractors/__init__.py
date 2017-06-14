import threading
from abc import ABCMeta, abstractmethod, abstractproperty
import rospy
from rospy import loginfo


class AbstractExtractor():
    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self.publisher = kwargs['publisher']
        self.register_me()

    @abstractmethod
    def extract(self, **kwargs):
        """

        :rtype: mario_messages.msg._RdfGraphStamped.RdfGraphStamped
        """

        pass

    def get_time(self):

        return rospy.Time().now()

    @abstractproperty
    def queue_size(self):
        pass

    @abstractproperty
    def rate(self):
        """

        :rtype: int or rospy.Rate
        """
        pass

    def run_forever(self):
        while not rospy.is_shutdown():
            result = (self.extract())
            if result:
                self.publisher.publish(result)
            # rate = rospy.Rate(self.rate) if isinstance(self.rate, int) else self.rate
            rospy.sleep(self.rate)

    def register_me(self):
        thread = threading.Thread(target=self.run_forever)
        thread.daemon = True
        thread.start()


class AbstractTopicExtractor(AbstractExtractor):
    def __init__(self, **kwargs):
        super(AbstractTopicExtractor, self).__init__(**kwargs)
        self.new = True

        def set_msg(data):
            self._msg = data
            self.new = True

        self.subscriber = rospy.Subscriber(self.from_channel, self.type, set_msg)

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

    def run_forever(self):
        while not rospy.is_shutdown():
            result = (self.extract())
            if result and self.new:
                self.publisher.publish(result)
                self.new = not self.only_new
            # rate = rospy.Rate(self.rate) if isinstance(self.rate, int) else self.rate
            rospy.sleep(self.rate)

    @property
    def message(self):
        try:
            return self._msg
        except AttributeError:
            while "_msg" not in self.__dict__ and not rospy.is_shutdown():
                loginfo("Waiting for first message...")
                rospy.sleep(self.rate)
            if not rospy.is_shutdown():
                return self._msg
