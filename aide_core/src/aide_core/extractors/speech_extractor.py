from speech_recognition import Microphone, Recognizer, UnknownValueError, RequestError

from aide_core.extractors import AbstractExtractor
from aide_core.namespaces import speech


class SpeechExtractor(AbstractExtractor):
    queue_size = 42

    def initialize(self):
        self.recognizer = Recognizer()
        self.adjusted = False

    # obtain audio from the microphone
    def loop(self):
        with Microphone() as source:
            if not self.adjusted:
                self.recognizer.adjust_for_ambient_noise(source)
                self.adjusted = True

            audio = self.recognizer.listen(source)

            text = self.process_audio(audio)

            if text:
                subj = speech._
                subj.speechContent = text
                self.publish(subj)

    def process_audio(self, audio):
        """
        Converts given audio to speech.

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
