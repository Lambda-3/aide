package aide_messages;

public interface AddApiRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddApiRequest";
  static final java.lang.String _DEFINITION = "string name\nstring file_content\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getFileContent();
  void setFileContent(java.lang.String value);
}
