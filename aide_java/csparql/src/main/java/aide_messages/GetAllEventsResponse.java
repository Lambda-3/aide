package aide_messages;

public interface GetAllEventsResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetAllEventsResponse";
  static final java.lang.String _DEFINITION = "Event[] events";
  java.util.List<aide_messages.Event> getEvents();
  void setEvents(java.util.List<aide_messages.Event> value);
}
