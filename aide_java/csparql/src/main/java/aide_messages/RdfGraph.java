package aide_messages;

public interface RdfGraph extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/RdfGraph";
  static final java.lang.String _DEFINITION = "RdfTriple[] triples";
  java.util.List<aide_messages.RdfTriple> getTriples();
  void setTriples(java.util.List<aide_messages.RdfTriple> value);
}
