package aide_messages;

public interface GetApiResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetApiResponse";
  static final java.lang.String _DEFINITION = "string file_content";
  java.lang.String getFileContent();
  void setFileContent(java.lang.String value);
}
