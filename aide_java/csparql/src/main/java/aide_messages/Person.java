package aide_messages;

public interface Person extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "aide_messages/Person";
  static final java.lang.String _DEFINITION = "string uuid\nstring name\nstring surname\nuint8 age\nstring workplace\nstring face_id\n";
  java.lang.String getUuid();
  void setUuid(java.lang.String value);
  java.lang.String getName();
  void setName(java.lang.String value);
  java.lang.String getSurname();
  void setSurname(java.lang.String value);
  byte getAge();
  void setAge(byte value);
  java.lang.String getWorkplace();
  void setWorkplace(java.lang.String value);
  java.lang.String getFaceId();
  void setFaceId(java.lang.String value);
}
