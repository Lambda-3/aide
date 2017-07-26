package aide_java.csparql.cep.exstrat;

import aide_java.csparql.cep.Event;
import eu.larkc.csparql.common.RDFTuple;

public class Continuous extends ExecutionStrategy {
    Continuous(Event event) {
    }

    @Override
    public boolean isEligibleForExecution(RDFTuple row) {
        // TODO Auto-generated method stub
        return true;
    }

}
