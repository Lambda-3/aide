package mario_messages;

public interface Event extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/Event";
  static final java.lang.String _DEFINITION = "string CONTINUOUS=\"continuous\"\nstring NEWONLY=\"newonly\"\nstring PRECISE=\"precise\"\nstring name\nstring[] params\nuint8 range\nuint8 step\nuint8 executionType\nstring sparqlWhere";
  static final java.lang.String CONTINUOUS = "\"continuous\"";
  static final java.lang.String NEWONLY = "\"newonly\"";
  static final java.lang.String PRECISE = "\"precise\"";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.util.List<java.lang.String> getParams();
  void setParams(java.util.List<java.lang.String> value);
  byte getRange();
  void setRange(byte value);
  byte getStep();
  void setStep(byte value);
  byte getExecutionType();
  void setExecutionType(byte value);
  java.lang.String getSparqlWhere();
  void setSparqlWhere(java.lang.String value);
}
