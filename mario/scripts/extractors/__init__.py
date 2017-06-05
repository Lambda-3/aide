from abc import ABCMeta, abstractmethod, abstractproperty


class AbstractExtractor():
    __metaclass__ = ABCMeta

    @abstractmethod
    def transform(self, message):
        """

        :rtype: mario_messages.msg.RdfTriple
        """

        pass

    @abstractproperty
    def from_channel(self):
        """

        :rtype: str 
        """
        pass

    @abstractproperty
    def type(self):
        """

        :rtype: genpy.Message
        """
        pass

    @property
    def queue_size(self):
        return 42

    def get_time(self):
        import rospy
        return rospy.Time().now()
