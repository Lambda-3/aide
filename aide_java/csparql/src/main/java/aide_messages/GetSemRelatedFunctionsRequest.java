package aide_messages;

public interface GetSemRelatedFunctionsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetSemRelatedFunctionsRequest";
  static final java.lang.String _DEFINITION = "string name\nuint8 top_k\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  byte getTopK();
  void setTopK(byte value);
}
