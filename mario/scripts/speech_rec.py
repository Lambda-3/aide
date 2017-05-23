#!/usr/bin/env python
import threading

import roslib
import rospy
import speech_recognition as sr
from mario_messages.srv import AddSpeech
from speech_recognition import AudioSource

if __name__ == "__main__":
    roslib.load_manifest('mario')
    r = sr.Recognizer()


class ServiceHandler:
    SERVICE_CHANNEL = 'mario/face_rec/speech_input'

    @staticmethod
    def get_service(callback):
        """
        Factory method that provides the ros service which is to be used by the
        rdf node.

        :param callback: Function which is to be executed when the service is
        called.
        :return:  the ros service.
        """
        return rospy.Service(ServiceHandler.SERVICE_CHANNEL,
                             AddSpeech,
                             callback)

    @staticmethod
    def call_service(speech):
        """
        Calls the provided service mentioned above.

        :type speech: str
        :param speech: String to be transmitted.
        """
        rospy.wait_for_service(ServiceHandler.SERVICE_CHANNEL)
        try:
            add_speech = rospy.ServiceProxy(ServiceHandler.SERVICE_CHANNEL,
                                            AddSpeech)
            add_speech(speech)
        except rospy.ServiceException:
            pass


def main():
    rospy.init_node('speech_recognition', anonymous=True)
    while not rospy.is_shutdown():
        # obtain audio from the microphone

        with sr.Microphone() as source:
            print("Say something!")
            # r.adjust_for_ambient_noise(source)
            audio = r.listen(source)
            print("Understood, processing")
            process_in_bg = threading.Thread(target=process_audio,
                                             args=(audio,))
            process_in_bg.start()


def process_audio(audio):
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
        time = rospy.Time.now()
        print time
        s = r.recognize_google(audio)
        ServiceHandler.call_service(s)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print(
            "Could not request results from Google Speech Recognition "
            "service; {0}".format(
                e))


if __name__ == '__main__':
    main()
