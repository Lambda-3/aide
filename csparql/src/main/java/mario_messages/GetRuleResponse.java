package mario_messages;

public interface GetRuleResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetRuleResponse";
  static final java.lang.String _DEFINITION = "Rule rule";
  mario_messages.Rule getRule();
  void setRule(mario_messages.Rule value);
}
