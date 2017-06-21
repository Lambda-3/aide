package mario_messages;

public interface RdfGraphStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/RdfGraphStamped";
  static final java.lang.String _DEFINITION = "RdfTripleStamped[] quadruples";
  java.util.List<mario_messages.RdfTripleStamped> getQuadruples();
  void setQuadruples(java.util.List<mario_messages.RdfTripleStamped> value);
}
