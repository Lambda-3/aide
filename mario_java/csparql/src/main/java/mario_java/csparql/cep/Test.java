package mario_java.csparql.cep;

import java.text.ParseException;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.apache.commons.lang.StringUtils;

public class Test {

    public static void main(String[] args) throws ParseException {
        String rawContent = "EXECUTE(util.type_of_argument) ?arg [RANGE 5s STEP 1337s] WHERE { mario:a mario:b ?arg.}'";
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
        System.out.println(step);
    }
}