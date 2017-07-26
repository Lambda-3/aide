package aide_messages;

public interface AddFunctionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddFunctionRequest";
  static final java.lang.String _DEFINITION = "Function function\n";
  aide_messages.Function getFunction();
  void setFunction(aide_messages.Function value);
}
