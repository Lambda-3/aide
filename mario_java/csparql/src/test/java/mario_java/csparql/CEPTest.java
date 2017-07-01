package mario_java.csparql;

import static org.junit.Assert.*;

import java.text.ParseException;

import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import mario_java.csparql.cep.CEP;


public class CEPTest {
    private CEP cep;

    @Test
    public void testSomething() {
        String test = "blablabla \n blablabla     ";
        String result = StringUtils.normalizeSpace(test);
        assertEquals(result, "blablabla blablabla");
    }
}
