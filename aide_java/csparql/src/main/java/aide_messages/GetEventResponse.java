package aide_messages;

public interface GetEventResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetEventResponse";
  static final java.lang.String _DEFINITION = "Event event";
  aide_messages.Event getEvent();
  void setEvent(aide_messages.Event value);
}
