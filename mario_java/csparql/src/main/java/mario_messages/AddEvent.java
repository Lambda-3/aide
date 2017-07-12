package mario_messages;

public interface AddEvent extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mario_messages/AddEvent";
  static final java.lang.String _DEFINITION = "uint8 CONTINUOUS=0\nuint8 NEWONLY=1\nuint8 PRECISE=2\nstring name\nstring[] params\nuint8 range\nuint8 step\nuint8 executionType\nstring sparqlWhere\n---\nbool success";
}
