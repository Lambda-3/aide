package mario_messages;

public interface TrackFaceRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/TrackFaceRequest";
  static final java.lang.String _DEFINITION = "string face_id\nbool is_tracked\ngeometry_msgs/Point position\n";
  java.lang.String getFaceId();
  void setFaceId(java.lang.String value);
  boolean getIsTracked();
  void setIsTracked(boolean value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
}
