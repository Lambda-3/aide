package mario_messages;

public interface GetAllFunctionsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllFunctionsResponse";
  static final java.lang.String _DEFINITION = "Function[] functions";
  java.util.List<mario_messages.Function> getFunctions();
  void setFunctions(java.util.List<mario_messages.Function> value);
}
