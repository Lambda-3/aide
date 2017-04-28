#!/usr/bin/env python
import os
import pickle
import threading

import cv2
import dlib
import numpy as np
import openface
import roslib
import rospy
import sensor_msgs.point_cloud2 as pc
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from mario.srv import TrackFace, ResetTrackFace
from sensor_msgs.msg import Image
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sklearn.mixture import GMM

# hack at time being
if __name__ == "__main__":
    np.set_printoptions(precision=2)
    roslib.load_manifest('mario')

    pc.read_points(None)


class ServiceHandler:
    """
    Provides the ros communication functionality for this node.
    """
    IMAGE_CHANNEL = "/camera/rgb/image_raw"
    SERVICE_CHANNEL = 'mario/face_rec/track_face'
    RESET_CHANNEL = 'mario/face_rec/reset'
    CLOUD_CHANNEL = "/camera/depth_registered/points"

    def __init__(self, face_tracker):

        self.img_subscriber = rospy.Subscriber(ServiceHandler.IMAGE_CHANNEL,
                                               Image,
                                               callback=self.set_img,
                                               queue_size=1)
        self.cloud_subscriber = rospy.Subscriber(ServiceHandler.CLOUD_CHANNEL,
                                                 PointCloud2,
                                                 callback=self.set_cloud,
                                                 queue_size=1)

        self.face_tracker = face_tracker

        self.sync_lock = threading.Lock()

        # register the reset service
        rospy.Service(ServiceHandler.RESET_CHANNEL, ResetTrackFace,
                      self.reset)

    def set_img(self, img):
        self._image = img

    def get_img(self):
        """
        Gets the camera image.

        If there's no image yet, waits until there is one.

        :return: Image where the faces shall be recognized.
        """
        while '_image' not in self.__dict__.keys():
            rospy.sleep(1)
        return self._image

    image = property(get_img, set_img)

    def set_cloud(self, cloud):
        self._cloud = cloud

    def get_cloud(self):
        """

        Gets the camera point cloud.

        If there's no cloud yet, waits until there is one.

        :return: Point cloud with 3D position of each pixel.
        """
        while '_cloud' not in self.__dict__.keys():
            rospy.sleep(1)
        return self._cloud

    cloud = property(get_cloud, set_cloud)

    @staticmethod
    def call_reset_service():
        try:
            reset = rospy.ServiceProxy(ServiceHandler.RESET_CHANNEL,
                                       ResetTrackFace)
            reset()
        except rospy.ServiceException:
            pass

    def reset(self, request):
        with self.sync_lock:
            remove_face = rospy.ServiceProxy(
                ServiceHandler.SERVICE_CHANNEL,
                TrackFace)
            for face in self.face_tracker.faces:
                if not face.is_unknown():
                    id = face.label
                    remove_face(id, False, None)
            self.face_tracker.faces = set()

    @staticmethod
    def get_service(callback):
        """
        Factory method that provides the ros service which is to be used by the
        rdf_handler node.

        :param callback: Function which is to be executed when the service is
        called.
        :return:  the ros service.
        """
        return rospy.Service(ServiceHandler.SERVICE_CHANNEL, TrackFace,
                             callback)

    def call_service(self, faces):
        """
        Calls the provided service mentioned above. Updates information about
        the tracked faces and their position in the rdf graph node.

        :type faces: set
        :param faces: Set of tracked faces to be updated in the rdf graph.
        """
        rospy.wait_for_service(ServiceHandler.SERVICE_CHANNEL)
        try:
            add_face = rospy.ServiceProxy(ServiceHandler.SERVICE_CHANNEL,
                                          TrackFace)

            for face in faces:
                if not face.is_unknown():
                    id = face.label
                    # add_face(id, True, Point(face.x, face.y, face.z))
        except rospy.ServiceException:
            pass

    def call_service_async(self):
        """
        Same as above, asynchronous.

        Creates a thread to call the service in background.
        :type faces: set
        :param faces: Set of tracked faces to be updated in the rdf graph.
        """
        threading.Thread(target=self.call_service,
                         args=(self.face_tracker.faces,)).start()

    def step(self):
        with self.sync_lock:
            image = self.image
            cloud = self.cloud
            self.face_tracker.process_image(image)
            set_faces_3d_coordinates(cloud, self.face_tracker.faces)
        self.call_service_async()


class Face:
    """
    This Small class represents a face in the video stream.

    It owns a label, a bounding box and dlib's correlation tracker to track
    the movement of the face in the stream.
    """

    def __init__(self, img, bounding_box):
        """
        Creates a Face object.

        Also starts tracking of the face in a given image with a given
        bounding box.
        :param img: Image to track the face.
        :param bounding_box: Face area to track.
        """
        self.bounding_box = bounding_box
        self.tracker = dlib.correlation_tracker()
        self.tracker.start_track(img, bounding_box)
        self.label = "Unknown"
        self.confidence = 1

    def update_tracker(self, img):
        """
        Updates the faces tracker.
        :param img: Image to update the tracker from.
        """
        self.tracker.update(img)

    def get_position(self):
        """
        Returns the position of the tracker.
        :return: Position of the tracker.
        """
        return self.tracker.get_position()

    def is_unknown(self):
        return self.label == "Unknown"


class FaceTracker:
    confidence_level = 0.50
    channel = "/mario/img"
    # Openface & dlib constants for the location of the models
    openface_dir = "/opt/openface"
    file_dir = os.path.dirname(os.path.realpath(__file__))
    model_dir = os.path.join(openface_dir, 'models')
    dlib_model_dir = os.path.join(model_dir, 'dlib')
    openface_model_dir = os.path.join(model_dir, 'openface')
    align = openface.AlignDlib(
        os.path.join(dlib_model_dir, "shape_predictor_68_face_landmarks.dat"))
    classifier_model = "/home/viktor/training-images/classifier.pkl"
    # Openface model
    net = openface.TorchNeuralNet(
        os.path.join(openface_model_dir, 'nn4.small2.v1.t7'))

    def __init__(self, visualize=False):
        self.bridge = CvBridge()
        self.vis = visualize
        if visualize:
            self.visual_publisher = rospy.Publisher("/mario/img", Image,
                                                    queue_size=1)
        self.faces = set()

    def recognize_faces(self):
        for face in self.faces:
            bb = face.bounding_box
            alignedFace = self.align.align(
                96,
                self.rgb_image,
                bb,
                landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
            if alignedFace is None:
                print("Unable to align face.")

            with open(self.classifier_model, 'r') as f:
                (le, clf) = pickle.load(f)

                rep = self.net.forward(alignedFace).reshape(1, -1)
                predictions = clf.predict_proba(rep).ravel()
                maxI = np.argmax(predictions)
                person = le.inverse_transform(maxI)
                confidence = predictions[maxI]
                print(
                    "Predict {} with {:.2f} confidence.".format(person,
                                                                confidence))
                if (confidence > self.confidence_level):
                    face.label = "{}".format(person)
                    face.confidence = confidence
                if isinstance(clf, GMM):
                    dist = np.linalg.norm(rep - clf.means_[maxI])
                    print("  + Distance from the mean: {}".format(dist))

    def get_biggest_bounding_box(self):
        """
        Detects the biggest bounding rectangle of a face.

        :return: Returns a dlib.rectangle which describes the bounding
        rectangle
        of a face.
        """
        # type: (object, Boolean) -> dlib.rectangle

        bb = self.align.getLargestFaceBoundingBox(self.rgb_image)

        if bb is None:
            print "bb is none!!!!!11111"
        else:
            print(
                "This bbox is centered at {}, {}".format(bb.center().x,
                                                         bb.center().y))

        return bb

    def detect_all_faces(self):
        """
        Detects all bounding boxes of all faces. Seen in the picture.

        :return: Returns all bounding boxes found in the picture.
        """
        bbs = self.align.getAllFaceBoundingBoxes(self.rgb_image)
        for bb in bbs:
            self.faces.add(Face(self.rgb_image, bb))
        return bbs

    def initialize(self):
        print("no faces detected yet. initializing face detection.")
        self.detect_all_faces()
        self.recognize_faces()

    def process_image(self, img):
        # type: (Image) -> None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)
            cv_image = None
        self.rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        if not len(self.faces):
            self.initialize()
        else:
            for face in self.faces:
                face.update_tracker(self.rgb_image)
        if self.vis:
            self.visualize()

    def visualize(self):
        for face in self.faces:
            left = int(face.get_position().left())
            right = int(face.get_position().right())
            top = int(face.get_position().top())
            bottom = int(face.get_position().bottom())

            # draw rectangle around the face
            self.rgb_image = cv2.rectangle(self.rgb_image, (left, top),
                                           (right, bottom),
                                           (0, 255, 0), 2)
            # put label
            self.rgb_image = cv2.putText(self.rgb_image,
                                         "{} @ {:.2f}".format(face.label,
                                                              face.confidence),
                                         (left, bottom),
                                         cv2.FONT_HERSHEY_SIMPLEX,
                                         1,
                                         (0, 255, 0), 1, cv2.LINE_AA)
        try:
            self.visual_publisher.publish(self.bridge.cv2_to_imgmsg(
                self.rgb_image, "rgb8"))
        except CvBridgeError as e:
            print(e)


def set_faces_3d_coordinates(cloud, faces):
    """
    For a given iterable of faces, sets the 3D coordinates of the faces
    centers from a given point cloud.

    Enriches each face with new attributes x, y and z.

    :type cloud: PointCloud2
    :param cloud: Point cloud from which the 3D coordinates shall be extracted.
    :type faces: iterable
    :param faces: Iterable of tracked faces.
    """
    uvs = []
    faces_list = list(faces)
    for face in faces_list:
        center = face.get_position().center()
        print type(center.x)
        uvs.append((center.x, center.y))

    points_of_interest = list(pc.read_points(cloud, uvs=uvs,
                                             field_names="x, y, z",
                                             skip_nans=True))

    for i in range(0, len(faces_list)):
        face = faces_list[i]
        face.x, face.y, face.z = points_of_interest[i]


def main():
    rospy.init_node('face_rec')
    tracker = FaceTracker(visualize=True)
    srv_handler = ServiceHandler(tracker)
    while not rospy.is_shutdown():
        srv_handler.step()
        rospy.Rate(8).sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
