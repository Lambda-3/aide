package aide_messages;

public interface AddSpeechRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddSpeechRequest";
  static final java.lang.String _DEFINITION = "string sentence\n";
  java.lang.String getSentence();
  void setSentence(java.lang.String value);
}
