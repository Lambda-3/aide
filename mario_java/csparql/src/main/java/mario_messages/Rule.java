package mario_messages;

public interface Rule extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/Rule";
  static final java.lang.String _DEFINITION = "string name\nstring description\nstring content";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  java.lang.String getContent();
  void setContent(java.lang.String value);
}
