package mario_java.csparql.cep.exstrat;

import java.lang.reflect.InvocationTargetException;

import eu.larkc.csparql.common.RDFTuple;
import mario_java.csparql.cep.Event;

public abstract class ExecutionStrategy {
    
    public abstract boolean isEligibleForExecution(RDFTuple row);

    public static ExecutionStrategy getStrategyFromType(ExecutionStrategyType type, Event event) {
        Class<? extends ExecutionStrategy> cls = type.getCorrespondingClass();
        try {
            return cls.getDeclaredConstructor(Event.class).newInstance(event);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
                | NoSuchMethodException | SecurityException e) {
            e.printStackTrace();
            throw new RuntimeException("Something somewhere went terribly wrong");
        }
    }

    @Override
    public String toString() {
        return this.getClass().getName();

    }
}
