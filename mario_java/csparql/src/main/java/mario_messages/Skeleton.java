package mario_messages;

public interface Skeleton extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/Skeleton";
  static final java.lang.String _DEFINITION = "uint8 id\ngeometry_msgs/Point head_position\n\n";
  byte getId();
  void setId(byte value);
  geometry_msgs.Point getHeadPosition();
  void setHeadPosition(geometry_msgs.Point value);
}
