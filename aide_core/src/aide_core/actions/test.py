from rospy import loginfo


def send_skype_message(to, content):
    """
    
    :param to: Person to send the message to.
    :type to: str
    :param content: Content of the message. 
    :type content: str
    """
    loginfo("Sending message to %s with content %s.", to, content)
