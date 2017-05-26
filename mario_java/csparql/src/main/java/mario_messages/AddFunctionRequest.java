package mario_messages;

public interface AddFunctionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddFunctionRequest";
  static final java.lang.String _DEFINITION = "Function function\n";
  mario_messages.Function getFunction();
  void setFunction(mario_messages.Function value);
}
