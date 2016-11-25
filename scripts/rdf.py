#!/usr/bin/env python
import os

import roslib
import rospy
from mario.msg import Person
from rdflib import ConjunctiveGraph, Literal, BNode

import face_rec
import speech_rec
from rdf_constants import (PATH, PERSONS, FACE, TYPES, SKEL_TRACKER,
                           BOOLEAN, NAME_QUERY, UUID_QUERY, DISTANCE_QUERY,
                           TRACKED_PERSONS_QUERY,
                           TRACKED_FACES_QUERY)
from rdf_constants import (id_from_face, face_from_id,
                           skeleton_from_id)
from rules import RuleHandler

roslib.load_manifest('mario')


# rdf path
# Constants for rdf namespaces.

# Constants for prepared SPARQL queries


class RdfGraph(ConjunctiveGraph):
    def __init__(self, path):
        super(RdfGraph, self).__init__('Sleepycat')
        self.graph = self
        self.open(path, create=(not os.path.isdir(path)))
        self._cleanup_functions = set()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._cleanup()
        self.graph.close()

    def add_cleanup_function(self, function):
        self._cleanup_functions.add(function)

    def _cleanup(self):
        for function in self._cleanup_functions:
            function()

    def add(self, triple_or_quad):
        # TODO: some on_add listener
        super(RdfGraph, self).add(triple_or_quad)

    def remove(self, triple_or_quad):
        # TODO: some on_remove listener
        super(RdfGraph, self).remove(triple_or_quad)


class ServiceHandler:
    def __init__(self, graph):
        """

        :type graph: RdfGraph
        """
        self.graph = graph

    def initialize_all_services(self):
        # rospy.Service('add_skeleton', AddSkeleton,
        #               (lambda req: self.add_skeleton(req.skel_id)))
        # rospy.Service('del_skeleton', DelSkeleton,
        #               (lambda req: self.del_skeleton(req.skel_id)))
        # rospy.Service('add_person', AddPerson,
        #               (lambda req: self.add_person_by_name(req.name)))
        # rospy.Service('link_skeleton', LinkSkeleton,
        #               (lambda req: self.link_skeleton(req.skel_id,
        #                                               req.name)))
        # rospy.Service('set_distance', SetDistance,
        #               (lambda req: self.set_distance(req.id,
        #                                              req.distance)))
        # rospy.Service('add_new_person', AddNewPerson,
        #               (lambda req: self.add_person(req.person)))
        # Add face recognition/tracking service
        face_rec.ServiceHandler.get_service(self.track_face)
        # Add speech recognition service
        speech_rec.ServiceHandler.get_service(self.add_speech)
        # TODO: Add skeleton tracking service

        self.graph.add_cleanup_function(self.cleanup_services)

    def add_speech(self, req):
        self.graph.add((BNode(), TYPES.speech, Literal(req.speech)))

    def track_face(self, req):
        """

        """
        face = face_from_id(req.face_id)
        if req.is_tracked:
            self.graph.add((face, FACE.tracked, BOOLEAN.true))
            # self.graph.remove((face, FACE.position, None))
            self.graph.set((face, FACE.position_x, Literal(req.pos_x)))
            self.graph.set((face, FACE.position_y, Literal(req.pos_y)))
            self.graph.set((face, FACE.position_z, Literal(req.pos_z)))

        else:
            self.graph.remove((face, None, None))
        return []

    def cleanup_services(self):
        self.graph.remove((None, SKEL_TRACKER.skeleton, None))
        self.graph.remove((None, SKEL_TRACKER.tracked, None))
        self.graph.remove((None, SKEL_TRACKER.distance_to_me, None))
        self.graph.remove((None, FACE.tracked, None))
        self.graph.remove((None, FACE.position_x, None))
        self.graph.remove((None, FACE.position_y, None))
        self.graph.remove((None, FACE.position_z, None))
        self.graph.remove((None, TYPES.speech, None))

    def add_person_by_name(self, name):
        print "adding person %s" % name
        person = BNode()
        self.graph.add((person, TYPES.type, TYPES.Person))
        self.graph.add((person, PERSONS.name, Literal(name)))

    def get_persons_names(self):
        query_result = self.graph.query(NAME_QUERY)

        result = set()
        for row in query_result:
            result.add(row['name'])

    def add_skeleton(self, skel_id):
        print "adding skeleton"
        skeleton = skeleton_from_id(skel_id)
        self.graph.add((skeleton, SKEL_TRACKER.tracked, BOOLEAN.true))
        print "added"

    def del_skeleton(self, skel_id):
        self.graph.remove((None, None, skel_id))
        self.graph.remove((skel_id, None, None))

    def link_skeleton(self, skel_id, name):
        skeleton = skeleton_from_id(skel_id)
        uuid = None
        result = self.graph.query(UUID_QUERY,
                                  initBindings={'queried_name': Literal(name)})
        for row in result:
            uuid = row['id']
        print uuid
        if self._is_skel_tracked(skel_id) and uuid:
            print("Linking Skeleton %d to %s" % (skel_id, name))
            self.graph.add((uuid, SKEL_TRACKER.skeleton, skeleton))
        else:
            print("Skeleton %d not tracked" % skel_id)

    def get_tracked_persons(self):
        query_result = self.graph.query(TRACKED_PERSONS_QUERY)

        result = set()
        for row in query_result:
            result.add(row['name'])

        return result

    def get_distance(self, name):
        result = self.graph.query(DISTANCE_QUERY,
                                  initBindings={'queried_name': Literal(name)})
        for row in result:
            return row['distance']

        # TODO: raise exception
        return 0

    def set_distance(self, skel_id, distance):
        if self._is_skel_tracked(skel_id):
            self.graph.set((skeleton_from_id(skel_id),
                            SKEL_TRACKER.distance_to_me,
                            Literal(distance)))
        return

    def _is_skel_tracked(self, skel_id):
        skeleton = skeleton_from_id(skel_id)
        skel_tracked = (skeleton, SKEL_TRACKER.tracked, BOOLEAN.true)
        return skel_tracked in self.graph

    def add_person(self, person):
        # type: (mario.msg.Person) -> []
        """

        :type person: Person
        """
        print("Adding Person: {} {}".format(person.name, person.surname))
        a = BNode()
        self.graph.add((a, PERSONS.name, Literal(person.name)))
        self.graph.add((a, PERSONS.age, Literal(person.age)))
        self.graph.add((a, PERSONS.workplace, Literal(person.workplace)))
        if person.face_id:
            self.graph.add((a, PERSONS.face, face_from_id(person.face_id)))
        return str(a)

    def get_tracked_faces(self):
        print("returning all tracked faces")

        query_result = self.graph.query(TRACKED_FACES_QUERY)

        result = set()
        for row in query_result:
            result.add(id_from_face(row['face_id']))

        return result


def main():
    rospy.init_node("rdf_handler")
    with RdfGraph(PATH) as graph:
        service_handler = ServiceHandler(graph)
        service_handler.initialize_all_services()

        rule_handler = RuleHandler(graph)
        rule_handler.load_all_rules()

        while not rospy.is_shutdown():
            # print graph.graph.serialize(format="n3")
            rule_handler.execute_rules()
            rospy.sleep(3)


if __name__ == "__main__":
    main()
