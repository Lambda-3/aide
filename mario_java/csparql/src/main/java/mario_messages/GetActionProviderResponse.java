package mario_messages;

public interface GetActionProviderResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetActionProviderResponse";
  static final java.lang.String _DEFINITION = "string file_content";
  java.lang.String getFileContent();
  void setFileContent(java.lang.String value);
}
