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
import com.hp.hpl.jena.datatypes.BaseDatatype.TypedValue;

import eu.larkc.csparql.common.RDFTable;
import eu.larkc.csparql.common.RDFTuple;
import eu.larkc.csparql.core.engine.CsparqlQueryResultProxy;

import org.apache.commons.lang.StringUtils;

public class ExecuteRule extends AbstractRule {
    private String functionName;
    private static final Log log = LogFactory.getLog(ExecuteRule.class);
    private static final TypeMapper typeMapper = TypeMapper.getInstance();
    private API api;
    private long lastExecution = 0;
    private int executionStep;
    private int epsilon = 5;
    private boolean newOnly;
    private boolean argless;

    ExecuteRule(String name, String content) throws ParseException {
        super(name, content);
        log.info("Created new ExecuteRule.");
    }

    @Override
    public void update(Observable o, Object arg) {
        long executionTime = System.currentTimeMillis();
        Gson gson = new Gson();
        RDFTable queryResult = (RDFTable) arg;

        // variable names of the query result
        Collection<String> names = queryResult.getNames();

        // for every row in the query result
        for (final RDFTuple t : queryResult) {
            // this is the execute when new only part
            // TODO: make more generic
            log.info("new only?: " + newOnly);
            if (this.newOnly && (lastExecution != 0 && (executionTime - lastExecution) < executionStep + epsilon)) {
                lastExecution = executionTime;
                log.info(String.format("Execution Step: %d; Delta:  %d", executionStep, executionTime - lastExecution));
                log.info("Delta too small. skipping");
                return;
            }

            lastExecution = executionTime;
            int i = 0;
            // gonna save the arguments here
            Map<String, Object> dict = new HashMap<>();
            // for every column in the result row
            if (!argless) {
                for (String name : names) {
                    String argument = t.get(i);
                    log.info("Argument: " + argument);
                    Object parsedArgument = null;

                    // split into argument value and type
                    // argumentParts[0] is value argumentParts[1] is type.
                    String[] argumentParts = argument.split("\\^\\^");

                    // if it has type
                    if (argumentParts.length > 1) {
                        // unquote argument
                        String value = argumentParts[0].substring(1, argumentParts[0].length() - 1);
                        String type = argumentParts[1];
                        // detect datatype
                        log.info("Argument value " + value + "; Argument Type: " + type);
                        RDFDatatype d = typeMapper.getTypeByName(type);
                        if (!(d == null)) {
                            // if datatype is known, parse it
                            log.info("RDF Datatype: " + d.toString());
                            parsedArgument = d.parse(value);
                            log.info("Parsed Argument" + parsedArgument.toString());
                        } else {
                            // if not, assume string
                            log.info("Unknown RDF datatype. Assuming string");
                            parsedArgument = value;

                        }
                        // if it doesn't have a type, just take it as it is
                    } else {
                        log.info("Argument has no type, assuming string.");
                        parsedArgument = argument.toString();
                    }

                    // save name->value pair
                    dict.put(name, parsedArgument);
                    ++i;
                }
            }
            // convert args to json
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
        rawContent = rawContent.trim();
        if (rawContent.endsWith("NEWONLY")) {
            this.newOnly = true;
            log.info("Rule Type: new only");
            rawContent = rawContent.substring(0, rawContent.length() - 7);
        }
        Matcher functionNameMatcher = Pattern.compile("EXECUTE?\\((.*?)\\)").matcher(rawContent);
        String functionName;

        // find function name
        if (functionNameMatcher.find()) {
            functionName = functionNameMatcher.group(1);
        } else {
            throw new ParseException("No function name found!", 0);
        }
        this.functionName = functionName;

        // find arguments
        Matcher argumentMatcher = Pattern.compile(functionName + "?\\)(.*?)[\\nF]").matcher(rawContent);
        List<String> arguments = new ArrayList<>();
        int i = 0;
        while (argumentMatcher.find()) {
            arguments.add(argumentMatcher.group(++i));
        }

        if (arguments.isEmpty()) {
            arguments.add("dummy_arg");
            this.argless = true;
        }

        Matcher stepMatcher = Pattern.compile("STEP ?(.*?)s").matcher(rawContent);
        int step;
        if (stepMatcher.find()) {
            String stringStep = stepMatcher.group(1);
            if (StringUtils.isNumeric(stringStep)) {
                step = Integer.parseInt(stringStep);
            } else {
                throw new ParseException("Step not numeric!", 0);
            }
        } else {
            throw new ParseException("No step found!", 0);
        }
        log.info("FunctionName: " + functionName);
        log.info("Arguments: " + arguments.toString());
        log.info("Step: " + step);
        this.executionStep = step * 1000;
        String fromStreamClause = "FROM STREAM <http://myexample.org/stream>";
        // assemble query
        String result = "REGISTER QUERY " + this.getName() + " as " + AbstractRule.KNOWN_PREFIXES;
        if (!argless) {
            result += rawContent.replaceAll("EXECUTE?\\((.*?)\\)", "SELECT").replace("[", fromStreamClause + " [");
        } else {
            log.info("argless");
            result += rawContent.replaceAll("EXECUTE?\\((.*?)\\)", "SELECT").replace("[",
                    " (\"asdf\" as ?dummy_arg) " + fromStreamClause + " [");
        }

        log.info("Constructed Query: " + result);
        return result;
    }

}
