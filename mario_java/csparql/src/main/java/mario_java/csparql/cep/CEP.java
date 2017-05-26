package mario_java.csparql.cep;

import java.text.ParseException;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import eu.larkc.csparql.cep.api.RdfStream;
import eu.larkc.csparql.core.engine.CsparqlEngine;
import eu.larkc.csparql.core.engine.CsparqlEngineImpl;

public class CEP extends CsparqlEngineImpl implements CsparqlEngine {
    private final Log log = LogFactory.getLog(CEP.class);
    private API api;
    private OutStream out;

    public CEP(API api, OutStream out) {
        log.info("Creating CEP instance");
        this.initialize(true);
        this.api = api;
        this.out = out;
    }

    public void addStream(RdfStream stream) {
        this.registerStream(stream);
    }

    API getApi() {
        return this.api;
    }

    OutStream getOutStream() {
        return this.out;
    }

    public boolean registerRule(String name, String content) throws ParseException {
        AbstractRule rule = AbstractRule.buildRule(name, content);
        rule.addMe(this);
        return true;
    }

}
