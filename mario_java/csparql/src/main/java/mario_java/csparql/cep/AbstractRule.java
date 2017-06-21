package mario_java.csparql.cep;

import java.text.ParseException;
import java.util.Observer;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import mario_java.csparql.ros.CallFunctionCallback;

public abstract class AbstractRule implements Observer {

    private static final Log log = LogFactory.getLog(CEP.class);
    private String name;
    private String content;

    static final String KNOWN_PREFIXES = "prefix classes: <http://prokyon:5000/mario/classes/> "
            + "prefix functions: <http://prokyon:5000/mario/functions/> "
            + "prefix mario: <http://prokyon:5000/mario/> " + "prefix persons: <http://prokyon:5000/mario/persons/> "
            + "prefix postures: <http://prokyon:5000/mario/postures/> "
            + "prefix properties: <http://prokyon:5000/mario/properties/> "
            + "prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> "
            + "prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> "
            + "prefix rules: <http://prokyon:5000/mario/rules/> "
            + "prefix skeletons: <http://prokyon:5000/mario/skeletons/> "
            + "prefix xml: <http://www.w3.org/XML/1998/namespace> " + "prefix xsd: <http://www.w3.org/2001/XMLSchema#> "
            + "PREFIX f: <http://larkc.eu/csparql/sparql/jena/ext#> ";

    AbstractRule(String name, String content) throws ParseException {
        log.info("Creating Rule...");
        log.info("Content is: " + content);
        this.name = name;
        this.content = this.parseContent(content);
    }

    protected abstract String parseContent(String rawContent) throws ParseException;

    protected abstract void addMe(CEP cep) throws ParseException;

    public String getContent() {
        return this.content;
    }

    public String getName() {
        return this.name;
    }

    static final AbstractRule buildRule(String name, String content) throws ParseException {
        log.info("Building Rule '" + name + "'.");
        if (name == null) {
            throw new IllegalArgumentException("Name must not be null!");
        }
        if (content == null) {
            throw new IllegalArgumentException("Rule content must not be null!");
        }

        if (content.contains("CONSTRUCT")) {
            return null;
        } else if (content.contains("EXECUTE")) {
            return new ExecuteRule(name, content);
        } else {
            throw new IllegalArgumentException("Unknown rule type!");
        }
    }

}
