package mario_java.csparql.cep.exstrat;

import eu.larkc.csparql.common.RDFTuple;
import mario_java.csparql.cep.Event;

public class Continuous extends ExecutionStrategy {
    Continuous(Event event) {
    }

    @Override
    public boolean isEligibleForExecution(RDFTuple row) {
        // TODO Auto-generated method stub
        return true;
    }

}
