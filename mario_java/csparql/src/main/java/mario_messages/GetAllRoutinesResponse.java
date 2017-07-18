package mario_messages;

public interface GetAllRoutinesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/GetAllRoutinesResponse";
  static final java.lang.String _DEFINITION = "Routine[] routines";
  java.util.List<mario_messages.Routine> getRoutines();
  void setRoutines(java.util.List<mario_messages.Routine> value);
}
