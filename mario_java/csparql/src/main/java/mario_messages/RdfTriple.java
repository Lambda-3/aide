package mario_messages;

public interface RdfTriple extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/RdfTriple";
  static final java.lang.String _DEFINITION = "string subject\nstring predicate\nstring object";
  java.lang.String getSubject();
  void setSubject(java.lang.String value);
  java.lang.String getPredicate();
  void setPredicate(java.lang.String value);
  java.lang.String getObject();
  void setObject(java.lang.String value);
}
