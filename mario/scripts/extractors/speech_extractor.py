import threading

import rospy
from mario_messages.msg import RdfGraphStamped, RdfTripleStamped
from speech_recognition import Microphone, Recognizer, UnknownValueError, RequestError

from apis.rdf_utils import Graph, Triple


class SpeechExtractor(object):
    queue_size = 42

    def __init__(self, publisher):
        self.publisher = publisher
        self.recognizer = Recognizer()
        thread = threading.Thread(target=self.run_forever)
        thread.daemon = True
        thread.start()

    # obtain audio from the microphone
    def run_forever(self):
        while not rospy.is_shutdown():
            with Microphone() as source:
                # r.adjust_for_ambient_noise(source)
                audio = self.recognizer.listen(source)
                text = self.process_audio(audio)
                if text:
                    self.publisher.publish(Graph(Triple("mario:kek", "properties:speech", text)))

    def process_audio(self, audio):
        """
        Converts given audio to speech.

        Calls the service defined in ServiceHandler to add the sentence to the
        rdf graph.
        :param audio: Audio snippet to be converted to text.
        """
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio,
            # key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            return self.recognizer.recognize_google(audio)
        except UnknownValueError:
            print("Google Speech Recognition could not understand audio")
        except RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))
