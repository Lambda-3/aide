package aide_messages;

public interface AddRuleRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddRuleRequest";
  static final java.lang.String _DEFINITION = "Routine routine\nEvent[] events\n";
  aide_messages.Routine getRoutine();
  void setRoutine(aide_messages.Routine value);
  java.util.List<aide_messages.Event> getEvents();
  void setEvents(java.util.List<aide_messages.Event> value);
}
