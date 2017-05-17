package mario_messages;

public interface TrackSkeletonRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/TrackSkeletonRequest";
  static final java.lang.String _DEFINITION = "bool is_tracked\nSkeleton skeleton\n";
  boolean getIsTracked();
  void setIsTracked(boolean value);
  mario_messages.Skeleton getSkeleton();
  void setSkeleton(mario_messages.Skeleton value);
}
