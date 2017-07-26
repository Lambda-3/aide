package aide_java.csparql.cep;

import java.text.ParseException;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Observable;
import java.util.Observer;

import org.apache.commons.lang3.StringUtils;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import com.hp.hpl.jena.datatypes.RDFDatatype;
import com.hp.hpl.jena.datatypes.TypeMapper;

import aide_java.csparql.cep.exstrat.ExecutionStrategy;
import aide_java.csparql.cep.exstrat.ExecutionStrategyType;
import eu.larkc.csparql.common.RDFTable;
import eu.larkc.csparql.common.RDFTuple;

public class Event implements Observer {

    private static final String DUMMY_ARG_NAME = "dummy_arg";
    private static final String RANGE = " [RANGE %ds STEP %ds] ";
    private static final String FROM_STREAM = " FROM STREAM <http://lambda3.org/aide/rdfstream> ";
    private static final String SELECT = " SELECT ";
    private static final String REGISTER_QUERY_AS = "REGISTER QUERY %s as ";
    private final Log log = LogFactory.getLog(this.getClass());
    private String name;
    private List<String> params;
    private int step;
    private int range;
    private ExecutionStrategy type;
    private String sparqlWhere;
    private EventStream out;

    // TODO gonna need to read that from somewhere
    private static final String KNOWN_PREFIXES = "prefix classes: <http://lambda3.org/aide/classes/> "
            + "prefix functions: <http://lambda3.org/aide/functions/> "
            + "prefix robot: <http://lambda3.org/aide/self/> " + "prefix persons: <http://lambda3.org/aide/persons/> "
            + "prefix postures: <http://lambda3.org/aide/postures/> "
            + "prefix properties: <http://lambda3.org/aide/properties/> "
            + "prefix rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> "
            + "prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#> "
            + "prefix rules: <http://lambda3.org/aide/rules/> "
            + "prefix skeletons: <http://lambda3.org/aide/skeletons/> "
            + "prefix xml: <http://www.w3.org/XML/1998/namespace> " + "prefix xsd: <http://www.w3.org/2001/XMLSchema#> "
            + "PREFIX f: <http://larkc.eu/csparql/sparql/jena/ext#> ";

    private static final TypeMapper typeMapper = TypeMapper.getInstance();

    public Event(String name, List<String> params, int range, int step, ExecutionStrategyType type, String sparqlWhere,
            EventStream out) throws ParseException {
        this.name = name;
        this.params = params;
        this.range = range;
        this.step = step;
        this.type = ExecutionStrategy.getStrategyFromType(type, this);
        this.sparqlWhere = StringUtils.normalizeSpace(sparqlWhere);
        this.out = out;
        this.log.info("Created " + this.toString());
    }

    public String getName() {
        return this.name;
    }

    public int getStep() {
        return step;
    }

    public int getRange() {
        return range;
    }

    public List<String> getParams() {
        return Collections.unmodifiableList(this.params);
    }

    String asSparql() {
        log.info("Creating SPARQL representation of event " + this.name + "...");
        StringBuilder result = new StringBuilder();
        result.append(String.format(REGISTER_QUERY_AS, this.name));
        result.append(KNOWN_PREFIXES);
        result.append(SELECT);
        if (this.params.size() > 0) {
            this.log.info("...appending parameters...");
            for (String param : this.params) {
                result.append(String.format(" ?%s ", param));
            }
        } else {
            this.log.info("...no parameters, appending dummy argument...");
            result.append(String.format(" (\"dummy\" as ?%s) ", DUMMY_ARG_NAME));
        }
        result.append(FROM_STREAM);
        result.append(String.format(RANGE, this.range, this.step));
        result.append(this.sparqlWhere);

        String resultString = StringUtils.normalizeSpace(result.toString());
        this.log.info("...created following representation:");
        this.log.info(resultString);
        return resultString;

    }

    @Override
    public void update(Observable o, Object arg) {
        RDFTable queryResult = (RDFTable) arg;
        this.log.info(queryResult.size());
        for (final RDFTuple row : queryResult) {
            if (type.isEligibleForExecution(row)) {

                this.out.put(this, this.rowAsMap(row, queryResult.getNames()));
            }
        }
    }

    private Map<String, Object> rowAsMap(RDFTuple row, Collection<String> names) {
        LinkedHashMap<String, Object> result = new LinkedHashMap<>();
        // if only dummy name in result table, then event is not parametrized
        if (names.size() == 1 && names.contains(DUMMY_ARG_NAME)) {
            return result;
        }
        // if not, proceed
        int i = 0;
        for (String name : names) {

            String param = row.get(i);

            Object parsedParam = null;
            // split into argument value and type
            // argumentParts[0] is value argumentParts[1] is type.
            String[] paramParts = param.split("\\^\\^");

            // if it has type
            if (paramParts.length > 1) {
                // unquote argument
                String value = paramParts[0].substring(1, paramParts[0].length() - 1);
                String type = paramParts[1];
                // detect datatype
                log.info("Param value " + value + "; Param type: " + type);
                RDFDatatype d = typeMapper.getTypeByName(type);
                if (!(d == null)) {
                    // if datatype is known, parse it
                    log.info("RDF Datatype: " + d.toString());
                    parsedParam = d.parse(value);
                    log.info("Parsed Argument: " + parsedParam.toString());
                } else {
                    // if not, assume string
                    log.info("Unknown RDF datatype. Assuming string");
                    parsedParam = value;

                }
                // if it doesn't have a type, just take it as it is
            } else {
                log.info("Argument has no type, assuming string.");
                parsedParam = param.toString();
            }
            result.put(name, parsedParam);

            ++i;
        }
        return result;
    }

    @Override
    public String toString() {
        return String.format("%s(params=%s, range=%d, step=%d executionType=%s, sparqlWhere=%s)", name,
                params.toString(), range, step, type, sparqlWhere);

    }

    @Override
    public boolean equals(Object obj) {
        log.info("Comparing " + this + "and " + obj);
        if (obj instanceof Event) {
            return this.name.equals(((Event) obj).name);
        } else {
            return false;
        }

    }

    @Override
    public int hashCode() {
        return this.name.hashCode();
    }
}
