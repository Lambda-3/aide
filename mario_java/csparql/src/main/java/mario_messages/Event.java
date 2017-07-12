package mario_messages;

public interface Event extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/Event";
  static final java.lang.String _DEFINITION = "string name\nstring params # Keyword Arguments, encoded as a json string";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getParams();
  void setParams(java.lang.String value);
}
