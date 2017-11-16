import rospy
from aide_messages.msg import ShortMessage

__pub = rospy.Publisher("/aide/outgoing_whatsapp_messages", ShortMessage, queue_size=42)
def send_whatsapp_message(to, content):
    """
    
    :param to: Whatsapp number to send message to.
    :type to: str
    :param content: Content to send.
    :type content: content.
    """
    msg = ShortMessage(type=ShortMessage.WhatsApp, other=to, content=content)
    __pub.publish(msg)
