package mario_messages;

public interface GetEventResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetEventResponse";
  static final java.lang.String _DEFINITION = "Event event";
  mario_messages.Event getEvent();
  void setEvent(mario_messages.Event value);
}
