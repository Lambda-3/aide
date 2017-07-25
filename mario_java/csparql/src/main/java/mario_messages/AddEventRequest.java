package mario_messages;

public interface AddEventRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddEventRequest";
  static final java.lang.String _DEFINITION = "Event event\n";
  mario_messages.Event getEvent();
  void setEvent(mario_messages.Event value);
}
