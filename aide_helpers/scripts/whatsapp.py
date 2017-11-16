#!/usr/bin/env python
import atexit
import threading

import roslib
import unicodedata
from rospy import loginfo

import rospy

from aide_messages.msg import ShortMessage, sys
from aide_core import credentials
from yowsup.layers.interface import YowInterfaceLayer, ProtocolEntityCallback
from yowsup.layers.protocol_chatstate.protocolentities import OutgoingChatstateProtocolEntity, ChatstateProtocolEntity
from yowsup.layers.protocol_messages.protocolentities import TextMessageProtocolEntity
from yowsup.layers.protocol_receipts.protocolentities import OutgoingReceiptProtocolEntity
from yowsup.layers.protocol_acks.protocolentities import OutgoingAckProtocolEntity
from yowsup.stacks import YowStackBuilder
from yowsup.layers import YowLayerEvent
from yowsup.layers.network import YowNetworkLayer

roslib.load_manifest('aide_helpers')

class AideRosLayer(YowInterfaceLayer):
    def __init__(self):
        super(AideRosLayer, self).__init__()
        loginfo("Initializing Ros services...")
        self.publisher = rospy.Publisher("/aide/incoming_whatsapp_messages", ShortMessage, queue_size=42)
        rospy.Subscriber("/aide/outgoing_whatsapp_messages", ShortMessage, self.send_msg)

    @ProtocolEntityCallback("message")
    def onMessage(self, message):
        # send receipt otherwise we keep receiving the same message over and over
        message.body = unicodedata.normalize("NFKD", message.body).encode('ascii','ignore')
        loginfo(u"Incoming message...\n{}".format(message))
        receipt = OutgoingReceiptProtocolEntity(message.getId(), message.getFrom(),
                                                'read', message.getParticipant())
        self.toLower(receipt)

        if message.getType() == "text":
            self.publisher.publish(ShortMessage("WhatApp", message.getFrom(), message.getBody()))

    @ProtocolEntityCallback("receipt")
    def onReceipt(self, entity):
        ack = OutgoingAckProtocolEntity(entity.getId(), "receipt", entity.getType(), entity.getFrom())
        self.toLower(ack)

    def send_msg(self, short_message):
        """

        :type short_message: ShortMessage
        """
        if not short_message.type:
            loginfo("Wrong message type!")
            return
        to = short_message.other if short_message.other else "4917645822486@s.whatsapp.net"
        if not to.endswith("@s.whatsapp.net"):
            to = to + "@s.whatsapp.net"

        if to.startswith("+"):
            to = to[1:]
        if to.startswith("00"):
            to = to[2:]
        # ...typing
        entity = OutgoingChatstateProtocolEntity(ChatstateProtocolEntity.STATE_TYPING, to)
        self.toLower(entity)

        rospy.sleep(0.5)

        entity = OutgoingChatstateProtocolEntity(ChatstateProtocolEntity.STATE_PAUSED, to)
        self.toLower(entity)

        outgoingMessageProtocolEntity = TextMessageProtocolEntity(short_message.content, to=to)
        self.toLower(outgoingMessageProtocolEntity)


def main():
    rospy.init_node("whatsapp_service")
    cred = credentials.WHATSAPP
    stackBuilder = YowStackBuilder()
    stack = (stackBuilder
             .pushDefaultLayers(True)
             .push(AideRosLayer)
             .build())
    loginfo("Stack built...")
    stack.setCredentials(cred)
    stack.broadcastEvent(YowLayerEvent(YowNetworkLayer.EVENT_STATE_CONNECT))  # sending the connect signal
    loginfo("Connected...")
    atexit.register(lambda: stack.broadcastEvent(YowLayerEvent(YowNetworkLayer.EVENT_STATE_DISCONNECT)))

    th = threading.Thread(target=stack.loop)
    th.daemon = True
    th.start()
    loginfo("Running in background.")
    loginfo("All done. spinning.")
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == "__main__":
    main()
