package aide_messages;

public interface AddEventRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/AddEventRequest";
  static final java.lang.String _DEFINITION = "Event event\n";
  aide_messages.Event getEvent();
  void setEvent(aide_messages.Event value);
}
