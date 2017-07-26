package aide_java.csparql.cep.exstrat;

import java.lang.reflect.InvocationTargetException;

import aide_java.csparql.cep.Event;
import eu.larkc.csparql.common.RDFTuple;

public abstract class ExecutionStrategy {

    public abstract boolean isEligibleForExecution(RDFTuple row);

    public static ExecutionStrategy getStrategyFromType(ExecutionStrategyType type, Event event) {
        Class<? extends ExecutionStrategy> cls = type.getCorrespondingClass();
        try {
            return cls.getDeclaredConstructor(Event.class).newInstance(event);
        } catch (NoSuchMethodException e) {
            try {
                return cls.newInstance();
            } catch (InstantiationException | IllegalAccessException e1) {
                e.printStackTrace();
                throw new RuntimeException("Something somewhere went terribly wrong");
            }
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException
                | SecurityException e) {
            e.printStackTrace();
            throw new RuntimeException("Something somewhere went terribly wrong");
        }
    }

    @Override
    public String toString() {
        return this.getClass().getName();

    }
}
