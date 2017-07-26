package aide_messages;

public interface Routine extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/Routine";
  static final java.lang.String _DEFINITION = "string name\nstring description\nstring trigger_name\nstring[] execution_steps #json encoded execution steps";
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getDescription();
  void setDescription(java.lang.String value);
  java.lang.String getTriggerName();
  void setTriggerName(java.lang.String value);
  java.util.List<java.lang.String> getExecutionSteps();
  void setExecutionSteps(java.util.List<java.lang.String> value);
}
