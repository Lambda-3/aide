package aide_messages;

public interface GetAllRoutinesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/GetAllRoutinesResponse";
  static final java.lang.String _DEFINITION = "Routine[] routines";
  java.util.List<aide_messages.Routine> getRoutines();
  void setRoutines(java.util.List<aide_messages.Routine> value);
}
