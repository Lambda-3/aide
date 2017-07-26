package aide_messages;

public interface GetFunctionResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetFunctionResponse";
  static final java.lang.String _DEFINITION = "Function function";
  aide_messages.Function getFunction();
  void setFunction(aide_messages.Function value);
}
