package mario_messages;

public interface GetAllEventsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllEventsResponse";
  static final java.lang.String _DEFINITION = "Event[] events";
  java.util.List<mario_messages.Event> getEvents();
  void setEvents(java.util.List<mario_messages.Event> value);
}
