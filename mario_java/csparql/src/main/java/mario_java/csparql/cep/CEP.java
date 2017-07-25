package mario_java.csparql.cep;

import java.text.ParseException;
import java.util.Collection;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Map;
import java.util.Set;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import eu.larkc.csparql.cep.api.RdfStream;
import eu.larkc.csparql.core.engine.CsparqlEngineImpl;
import eu.larkc.csparql.core.engine.CsparqlQueryResultProxy;

public class CEP extends CsparqlEngineImpl {
    private final Log log = LogFactory.getLog(CEP.class);
    private API api;
    private EventStream out;
    private Map<Event, String> eventsToIds = new LinkedHashMap<>();

    public CEP(API api) {
        log.info("Creating CEP instance");
        this.initialize(true);
        this.api = api;
    }

    public void addStream(RdfStream stream) {
        this.registerStream(stream);
    }

    API getApi() {
        return this.api;
    }

    EventStream getOutStream() {
        return this.out;
    }

    public boolean registerRule(String name, String content) throws ParseException {
        AbstractRule rule = AbstractRule.buildRule(name, content);
        rule.addMe(this);
        return true;
    }

    public boolean registerEvent(Event event) throws ParseException {
        if (eventsToIds.containsKey(event)) {
            log.info("Event with this name already exists!");
            unregisterQuery(eventsToIds.get(event));
        }
        log.info(eventsToIds.size());
        log.info(eventsToIds);
        CsparqlQueryResultProxy result = registerQuery(event.asSparql(), false);
        result.addObserver(event);
        eventsToIds.put(event, result.getId());
        return true;
    }

}
