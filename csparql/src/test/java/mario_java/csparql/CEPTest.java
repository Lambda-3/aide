package mario_java.csparql;

import static org.junit.Assert.*;

import java.text.ParseException;
import java.util.Observable;
import java.util.Observer;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import eu.larkc.csparql.cep.api.RdfQuadruple;
import eu.larkc.csparql.cep.api.RdfStream;
import eu.larkc.csparql.common.RDFTuple;
import eu.larkc.csparql.core.engine.ConsoleFormatter;
import eu.larkc.csparql.core.engine.CsparqlEngine;
import eu.larkc.csparql.core.engine.CsparqlEngineImpl;
import eu.larkc.csparql.core.engine.CsparqlQueryResultProxy;
import mario_java.csparql.cep.API;
import mario_java.csparql.cep.CEP;
import mario_java.csparql.cep.OutStream;

public class CEPTest {
    private CEP cep;

    @Before
    public void setUp() {

        this.cep = new CEP(new API() {

            @Override
            public void call(String functionName, String args) {
                // TODO Auto-generated method stub

            }
        }, new OutStream() {

            @Override
            public void put(RDFTuple triple) {
                // TODO Auto-generated method stub

            }
        });

    }

    @Test(expected = IllegalArgumentException.class)
    public void CepAddRuleShouldThrowIllegalArgument() throws ParseException {
        cep.registerRule("thomas", null);
    }

    @Test
    public void testSomething() {
        CsparqlEngineImpl engine = new CsparqlEngineImpl();
        engine.initialize(true);
        System.out.println("humm");
        RdfStream stream = new RdfStream("http://myexample.org/stream");
        engine.registerStream(stream);
        String query = "REGISTER QUERY WhoLikesWhat as SELECT ?s ?p ?o  FROM STREAM <http://myexample.org/stream> [RANGE 5s STEP 1s] WHERE { ?s ?p ?o }";
        // engine.registerStream(null);
        try {
            CsparqlQueryResultProxy registerQuery = engine.registerQuery(query, false);
            registerQuery.addObserver(new Observer() {

                @Override
                public void update(Observable o, Object arg) {
                    System.out.println("WAAAAT");
                }
            });
        } catch (ParseException e) {
            throw new RuntimeException(e);
        }
        stream.put(new RdfQuadruple("http://myexample.org/stream/wat", "http://myexample.org/stream/wat", "http://myexample.org/stream/wat", System.currentTimeMillis()));

        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));
        stream.put(new RdfQuadruple("wat", "wat", "wat", System.currentTimeMillis()));

    }
}
