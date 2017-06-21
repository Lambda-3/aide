package mario_messages;

public interface RdfGraph extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/RdfGraph";
  static final java.lang.String _DEFINITION = "RdfTriple[] triples";
  java.util.List<mario_messages.RdfTriple> getTriples();
  void setTriples(java.util.List<mario_messages.RdfTriple> value);
}
