package mario_messages;

public interface GetFunctionResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetFunctionResponse";
  static final java.lang.String _DEFINITION = "Function function";
  mario_messages.Function getFunction();
  void setFunction(mario_messages.Function value);
}
