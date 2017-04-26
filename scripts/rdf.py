#!/usr/bin/env python

import roslib
import rospy
from mario.msg._RdfTriple import RdfTriple

roslib.load_manifest('mario')

import face_rec
import speech_rec
import config

from rdflib import Literal, BNode
from rdf_no_ros import FakeApi
from rdf_constants import (PATH, FACE, TYPES, SKEL_TRACKER, BOOLEAN)
from rdf_constants import (face_from_id, skeleton_from_id)
from rest import RestApi
from rules import RuleHandler
from skeleton_rec import ServiceHandler as SkeletonHandler


class ServiceHandler:
    def __init__(self, graph):
        """

        :type graph: RdfGraph
        """
        self.graph = graph

    def initialize_all_services(self):
        # Add face recognition/tracking service
        face_rec.ServiceHandler.get_service(self.track_face)
        # Add speech recognition service
        speech_rec.ServiceHandler.get_service(self.add_speech)
        # Add Skeleton tracking service
        SkeletonHandler.get_service(self.track_skeleton)

        self.graph.add_cleanup_function(self.cleanup_services)

    def track_skeleton(self, req):
        """

        :type req: mario.srv._TrackSkeleton.TrackSkeletonRequest
        """
        skeleton = skeleton_from_id(req.skeleton.id)
        if req.is_tracked:
            self.graph.add((skeleton, SKEL_TRACKER.tracked, BOOLEAN.true))
            self.graph.set((skeleton, SKEL_TRACKER.position_x, Literal(
                req.skeleton.head_position.x)))
            self.graph.set((skeleton, SKEL_TRACKER.position_y, Literal(
                req.skeleton.head_position.y)))
            self.graph.set((skeleton, SKEL_TRACKER.position_z,
                            Literal(-req.skeleton.head_position.z)))
        else:
            self.graph.remove((None, None, skeleton))
            self.graph.remove((skeleton, None, None))

    def add_speech(self, req):
        """

        :type req: mario.srv._AddSpeech.AddSpeechRequest
        """
        self.graph.add((BNode(), TYPES.speech, Literal(req.sentence)))

    def track_face(self, req):
        """
        :type req: mario.srv._TrackFace.TrackFaceRequest

        """
        face = face_from_id(req.face_id)
        if req.is_tracked:
            self.graph.add((face, FACE.tracked, BOOLEAN.true))
            self.graph.set((face, FACE.position_x, Literal(req.position.x)))
            self.graph.set((face, FACE.position_y, Literal(req.position.y)))
            self.graph.set((face, FACE.position_z, Literal(req.position.z)))

        else:
            self.graph.remove((face, None, None))
        return []

    def cleanup_services(self):
        self.graph.remove((None, SKEL_TRACKER.position_x, None))
        self.graph.remove((None, SKEL_TRACKER.position_y, None))
        self.graph.remove((None, SKEL_TRACKER.position_z, None))
        self.graph.remove((None, SKEL_TRACKER.tracked, None))
        self.graph.remove((None, FACE.tracked, None))
        self.graph.remove((None, FACE.position_x, None))
        self.graph.remove((None, FACE.position_y, None))
        self.graph.remove((None, FACE.position_z, None))
        self.graph.remove((None, TYPES.speech, None))
        self.graph.remove((None, None, None))


class SimpleCEPAndKB:
    def __init__(self, kb):
        self.graph = kb
        rospy.Subscriber("/mario/rdf", RdfTriple, self.create_triple_from_msg)

    def create_triple_from_msg(self, msg):
        """

        :type msg: RdfTriple
        """
        result = self.graph.query(
            """CONSTRUCT {{ {} {} {} }} WHERE {{ }}""".format(msg.subject, msg.predicate, msg.object))

        for row in result:
            self.graph.set(row)

    def spin(self):
        # service_handler = ServiceHandler(graph)
        # service_handler.initialize_all_services()

        # api = RestApi(graph)
        # api.run()
        while not rospy.is_shutdown():
            # print graph.graph.serialize(format="turtle")
            self.graph.execute_rules()
            self.graph.pprint()
            rospy.sleep(3)


def main():
    rospy.init_node("rdf_handler")
    with RuleHandler(config.RDF_PATH, FakeApi, config.ONTHOLOGY_PATH) as cep_and_kb:
        cep_and_kb.add_cleanup_function(lambda: cep_and_kb.remove((None, None, None)))
        cep = SimpleCEPAndKB(cep_and_kb)
        cep.spin()


if __name__ == "__main__":
    main()
