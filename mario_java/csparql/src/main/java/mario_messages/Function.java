package mario_messages;

public interface Function extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/Function";
  static final java.lang.String _DEFINITION = "string name\nstring doc\nstring[] args\nstring body";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDoc();
  void setDoc(java.lang.String value);
  java.util.List<java.lang.String> getArgs();
  void setArgs(java.util.List<java.lang.String> value);
  java.lang.String getBody();
  void setBody(java.lang.String value);
}
