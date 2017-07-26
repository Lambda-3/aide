package aide_java.csparql.cep;

import java.util.Map;


public interface EventStream {
    public void put(Event event, Map<String, Object> params);
}
