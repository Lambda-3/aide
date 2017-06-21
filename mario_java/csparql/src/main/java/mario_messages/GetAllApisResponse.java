package mario_messages;

public interface GetAllApisResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllApisResponse";
  static final java.lang.String _DEFINITION = "string[] api_names";
  java.util.List<java.lang.String> getApiNames();
  void setApiNames(java.util.List<java.lang.String> value);
}
