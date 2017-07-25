package mario_messages;

public interface AddRuleRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddRuleRequest";
  static final java.lang.String _DEFINITION = "Routine routine\nEvent[] events\n";
  mario_messages.Routine getRoutine();
  void setRoutine(mario_messages.Routine value);
  java.util.List<mario_messages.Event> getEvents();
  void setEvents(java.util.List<mario_messages.Event> value);
}
