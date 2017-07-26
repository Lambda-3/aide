package aide_messages;

public interface AddApiResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddApiResponse";
  static final java.lang.String _DEFINITION = "bool success\nstring errors";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getErrors();
  void setErrors(java.lang.String value);
}
