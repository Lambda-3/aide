package mario_java.csparql.cep.exstrat;

import eu.larkc.csparql.common.RDFTuple;

public abstract class ExecutionStrategy {

    public abstract boolean isEligibleForExecution(RDFTuple row);

    public static ExecutionStrategy getStrategyFromType(ExecutionStrategyType type) {
        Class<? extends ExecutionStrategy> cls = type.getCorrespondingClass();
        try {
            return cls.newInstance();
        } catch (InstantiationException | IllegalAccessException e) {
            e.printStackTrace();
            throw new RuntimeException("Something somewhere went terribly wrong");
        }
    }
}
