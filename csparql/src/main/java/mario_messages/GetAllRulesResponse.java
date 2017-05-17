package mario_messages;

public interface GetAllRulesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllRulesResponse";
  static final java.lang.String _DEFINITION = "Rule[] rules";
  java.util.List<mario_messages.Rule> getRules();
  void setRules(java.util.List<mario_messages.Rule> value);
}
