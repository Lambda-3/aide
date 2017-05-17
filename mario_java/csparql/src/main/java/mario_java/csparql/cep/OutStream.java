package mario_java.csparql.cep;

import eu.larkc.csparql.common.RDFTuple;

public interface OutStream {
    public void put(RDFTuple triple);
}
