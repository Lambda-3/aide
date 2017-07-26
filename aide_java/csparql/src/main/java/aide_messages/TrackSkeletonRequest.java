package aide_messages;

public interface TrackSkeletonRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/TrackSkeletonRequest";
  static final java.lang.String _DEFINITION = "bool is_tracked\nSkeleton skeleton\n";
  boolean getIsTracked();
  void setIsTracked(boolean value);
  aide_messages.Skeleton getSkeleton();
  void setSkeleton(aide_messages.Skeleton value);
}
