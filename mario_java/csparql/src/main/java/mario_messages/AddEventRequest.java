package mario_messages;

public interface AddEventRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddEventRequest";
  static final java.lang.String _DEFINITION = "uint8 CONTINUOUS=0\nuint8 NEWONLY=1\nuint8 PRECISE=2\nstring name\nstring[] params\nuint8 range\nuint8 step\nuint8 executionType\nstring sparqlWhere\n";
  static final byte CONTINUOUS = 0;
  static final byte NEWONLY = 1;
  static final byte PRECISE = 2;
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
