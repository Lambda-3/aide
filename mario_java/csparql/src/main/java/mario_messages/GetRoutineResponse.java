package mario_messages;

public interface GetRoutineResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetRoutineResponse";
  static final java.lang.String _DEFINITION = "Routine routine";
  mario_messages.Routine getRoutine();
  void setRoutine(mario_messages.Routine value);
}
