package aide_messages;

public interface CallFunctionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/CallFunctionRequest";
  static final java.lang.String _DEFINITION = "string func_name\nstring kwargs # Keyword Arguments, encoded as a json string\n";
  java.lang.String getFuncName();
  void setFuncName(java.lang.String value);
  java.lang.String getKwargs();
  void setKwargs(java.lang.String value);
}
