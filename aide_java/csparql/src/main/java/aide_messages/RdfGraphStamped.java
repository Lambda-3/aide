package aide_messages;

public interface RdfGraphStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/RdfGraphStamped";
  static final java.lang.String _DEFINITION = "RdfTripleStamped[] quadruples";
  java.util.List<aide_messages.RdfTripleStamped> getQuadruples();
  void setQuadruples(java.util.List<aide_messages.RdfTripleStamped> value);
}
