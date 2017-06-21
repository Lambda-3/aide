package mario_java.csparql.cep;

import java.util.List;

import eu.larkc.csparql.common.RDFTuple;

public interface OutStream {
    public void put(List<RDFTuple> triples);
}
