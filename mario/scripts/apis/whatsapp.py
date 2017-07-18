from yowsup.layers.interface import YowInterfaceLayer, ProtocolEntityCallback
from yowsup.layers.protocol_messages.protocolentities import TextMessageProtocolEntity
from yowsup.layers.protocol_receipts.protocolentities import OutgoingReceiptProtocolEntity
from yowsup.layers.protocol_acks.protocolentities import OutgoingAckProtocolEntity
from yowsup.stacks import YowStackBuilder
from yowsup.layers import YowLayerEvent
from yowsup.layers.network import YowNetworkLayer


class MarioLayer(YowInterfaceLayer):
    @ProtocolEntityCallback("message")
    def onMessage(self, messageProtocolEntity):
        # send receipt otherwise we keep receiving the same message over and over

        if True:
            receipt = OutgoingReceiptProtocolEntity(messageProtocolEntity.getId(), messageProtocolEntity.getFrom(),
                                                    'read', messageProtocolEntity.getParticipant())

            outgoingMessageProtocolEntity = TextMessageProtocolEntity(
                messageProtocolEntity.getBody(),
                to=messageProtocolEntity.getFrom())
            print (messageProtocolEntity.getFrom())
            self.toLower(receipt)
            self.toLower(outgoingMessageProtocolEntity)
            self.send_msg()

    @ProtocolEntityCallback("receipt")
    def onReceipt(self, entity):
        ack = OutgoingAckProtocolEntity(entity.getId(), "receipt", entity.getType(), entity.getFrom())
        self.toLower(ack)

    def send_msg(self):
        outgoingMessageProtocolEntity = TextMessageProtocolEntity(
            "it works", to="4917645822486@s.whatsapp.net")
        self.toLower(outgoingMessageProtocolEntity)


def on_receive_message(callback):
    pass


def send_message(to, message_body):
    pass


def _setup():
    credentials = ("491603522638", "rK3gPc0qP4OLpF4pWnuqCLTBoJU=")  # replace with your phone and password
    stackBuilder = YowStackBuilder()

    stack = stackBuilder \
        .pushDefaultLayers(True) \
        .push(MarioLayer) \
        .build()
    mario_layer = stack.getLayer(8)
    stack.setCredentials(credentials)
    stack.broadcastEvent(YowLayerEvent(YowNetworkLayer.EVENT_STATE_CONNECT))  # sending the connect signal
    mario_layer.send_msg()
    # stack.broadcastEvent(YowLayerEvent(YowNetworkLayer.EVENT_STATE_DISCONNECT))
    stack.loop()  # this is the program mainloop


if __name__ == "__main__":
    _setup()
