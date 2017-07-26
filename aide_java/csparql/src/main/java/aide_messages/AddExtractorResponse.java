package aide_messages;

public interface AddExtractorResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddExtractorResponse";
  static final java.lang.String _DEFINITION = "bool success\nstring errors";
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getErrors();
  void setErrors(java.lang.String value);
}
