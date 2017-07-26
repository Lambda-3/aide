package aide_messages;

public interface GetAllActionProvidersResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetAllActionProvidersResponse";
  static final java.lang.String _DEFINITION = "string[] api_names";
  java.util.List<java.lang.String> getApiNames();
  void setApiNames(java.util.List<java.lang.String> value);
}
