package mario_messages;

public interface GetAllExtractorsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllExtractorsResponse";
  static final java.lang.String _DEFINITION = "string[] extractor_names";
  java.util.List<java.lang.String> getExtractorNames();
  void setExtractorNames(java.util.List<java.lang.String> value);
}
