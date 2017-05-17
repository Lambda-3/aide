package mario_messages;

public interface AddRuleRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddRuleRequest";
  static final java.lang.String _DEFINITION = "Rule rule\n";
  mario_messages.Rule getRule();
  void setRule(mario_messages.Rule value);
}
