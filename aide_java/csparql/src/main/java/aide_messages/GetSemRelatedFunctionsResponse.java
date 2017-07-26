package aide_messages;

public interface GetSemRelatedFunctionsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetSemRelatedFunctionsResponse";
  static final java.lang.String _DEFINITION = "Function[] functions";
  java.util.List<aide_messages.Function> getFunctions();
  void setFunctions(java.util.List<aide_messages.Function> value);
}
