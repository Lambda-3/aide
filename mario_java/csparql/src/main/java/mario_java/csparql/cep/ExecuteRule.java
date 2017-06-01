package mario_java.csparql.cep;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Observable;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import com.google.gson.Gson;
import com.hp.hpl.jena.datatypes.RDFDatatype;
import com.hp.hpl.jena.datatypes.TypeMapper;
import com.hp.hpl.jena.rdf.model.Literal;
import com.hp.hpl.jena.rdf.model.impl.PropertyImpl;
import com.hp.hpl.jena.rdf.model.impl.ResourceImpl;
import com.hp.hpl.jena.rdf.model.impl.StatementImpl;

import eu.larkc.csparql.common.RDFTable;
import eu.larkc.csparql.common.RDFTuple;
import eu.larkc.csparql.core.engine.CsparqlQueryResultProxy;

public class ExecuteRule extends AbstractRule {
    private String functionName;
    private static final Log log = LogFactory.getLog(ExecuteRule.class);
    private static final TypeMapper typeMapper = TypeMapper.getInstance();
    private API api;
    private long lastExecution = 0;
    private int executionStep = 10000;
    private int epsilon = 10;

    ExecuteRule(String name, String content) throws ParseException {
        super(name, content);
        log.info("Created new ExecuteRule.");
    }

    @Override
    public void update(Observable o, Object arg) {
        long executionTime = System.currentTimeMillis();

        RDFTable q = (RDFTable) arg;
        Collection<String> names = q.getNames();
        Gson gson = new Gson();
        for (final RDFTuple t : q) {
            if (lastExecution != 0 && (executionTime - lastExecution) < executionStep + epsilon) {
                lastExecution = executionTime;
                log.info("Delta too small. skipping");
                return;
            }

            lastExecution = executionTime;
            int i = 0;
            Map<String, Object> dict = new HashMap<>();
            for (String name : names) {
                String argument = t.get(i);
                log.info("Argument: " + argument);
                Object parsedArgument = null;
                // argumentParts[0] is value argumentParts[1] is type.
                String[] argumentParts = argument.split("\\^\\^");
                if (argumentParts.length > 1) {
                    String value = argumentParts[0].substring(1, argumentParts[0].length() - 1);
                    String type = argumentParts[1];
                    log.info("Argument value " + value + "; Argument Type: " + type);
                    RDFDatatype d = typeMapper.getTypeByName(type);
                    if (!(d == null)) {
                        log.info("RDF Datatype: " + d.toString());
                        parsedArgument = d.parse(value);
                        log.info("Parsed Argument" + parsedArgument.toString());
                    } else {
                        log.info("Unknown RDF datatype. Assuming string");
                        parsedArgument = argument;
                    }
                } else {
                    log.info("Argument has no type, assuming string.");
                    parsedArgument = argument;
                }

                dict.put(name, parsedArgument);
                ++i;
            }
            String args = gson.toJson(dict);
            log.info(String.format("Function name: %s, args: %s", this.functionName, args));
            this.api.call(this.functionName, args);
        }

    }

    @Override
    public void addMe(CEP cep) throws ParseException {
        String query = this.getContent();
        CsparqlQueryResultProxy resultProxy = cep.registerQuery(query, false);
        this.api = cep.getApi();
        resultProxy.addObserver(this);

    }

    protected String parseContent(String rawContent) throws ParseException {
        Matcher functionNameMatcher = Pattern.compile("EXECUTE?\\((.*?)\\)").matcher(rawContent);
        String functionName;
        if (functionNameMatcher.find()) {
            functionName = functionNameMatcher.group(1);
        } else {
            throw new ParseException("No function name found!", 0);
        }
        this.functionName = functionName;
        Matcher argumentMatcher = Pattern.compile(functionName + "?\\)(.*?)[\\nF]").matcher(rawContent);
        List<String> arguments = new ArrayList<>();
        int i = 0;
        while (argumentMatcher.find()) {
            arguments.add(argumentMatcher.group(++i));
        }
        log.info("FunctionName: " + functionName);
        log.info("Arguments: " + arguments.toString());
        String fromStreamClause = "FROM STREAM <http://myexample.org/stream>";
        String result = "REGISTER QUERY " + this.getName() + " as " + AbstractRule.KNOWN_PREFIXES
                + rawContent.replaceAll("EXECUTE?\\((.*?)\\)", "SELECT").replace("[", fromStreamClause + " [");
        log.info("Constructed Query: " + result);
        return result;
    }

}
