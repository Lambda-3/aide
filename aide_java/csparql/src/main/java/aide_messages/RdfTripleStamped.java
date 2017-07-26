package aide_messages;

public interface RdfTripleStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/RdfTripleStamped";
  static final java.lang.String _DEFINITION = "string subject\nstring predicate\nstring object\ntime stamp";
  java.lang.String getSubject();
  void setSubject(java.lang.String value);
  java.lang.String getPredicate();
  void setPredicate(java.lang.String value);
  java.lang.String getObject();
  void setObject(java.lang.String value);
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
}
