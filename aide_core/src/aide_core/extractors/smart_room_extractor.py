import json

from std_msgs.msg import String

from aide_core.extractors import AbstractTopicExtractor
from aide_core.namespaces import smartRoom

class SmartRoomExtractor(AbstractTopicExtractor):
    from_channel = "/aide/building"
    type = String
    queue_size = 43
    rate = 2

    def extract_from_message(self, message):
        msg = json.loads(message.data)
        subj = smartRoom.building
        subj.a = "Building"
        subj.people = msg['people']
        subj.isEmpty = msg['isEmpty']
        subj.controlOverride = msg['controlOverride']

        subjs = [subj]
        for door in msg['doors']:

            door_subj = smartRoom._
            door_subj.a = "Door"
            door_subj.state = door['state']
            subjs.append(door_subj)

        for room in msg['rooms']:
            room_subj = smartRoom._
            room_subj.a = "Room"

            room_subj.alias = room['roomAlias']
            room_subj.roomNumber = room['roomNr']
            room_subj.fireAlarmTriggered = room['fireAlarmTriggered']

            subj["room%s" % room['roomNr']] = room_subj
            subjs.append(room_subj)




        return tuple(subjs)
