package aide_messages;

public interface GetRoutineResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetRoutineResponse";
  static final java.lang.String _DEFINITION = "Routine routine";
  aide_messages.Routine getRoutine();
  void setRoutine(aide_messages.Routine value);
}
