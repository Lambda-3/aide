package aide_java.csparql.cep.exstrat;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import aide_java.csparql.cep.Event;
import eu.larkc.csparql.common.RDFTuple;

public class NewOnly extends ExecutionStrategy {
    // private long lastExecution = 0;
    private int executionStep;
    private int epsilon = 5;
    private final Log log = LogFactory.getLog(this.getClass());
    private Map<Integer, Long> lastExecutions = new LinkedHashMap<>();

    NewOnly(Event event) {
        this.executionStep = event.getStep() * 1000;
        this.log.info("Execution step: " + executionStep);
    }

    @Override
    public boolean isEligibleForExecution(RDFTuple row) {
        int hash = getHash(row);
        long lastExecution = lastExecutions.containsKey(hash) ? lastExecutions.get(hash) : 0;
        log.info(String.format("Last execution time: %d", lastExecution));
        long executionTime = System.currentTimeMillis();
        boolean eligible;
        log.info(String.format("Execution Step: %d; Delta:  %d", executionStep, executionTime - lastExecution));
        if (lastExecution != 0 && (executionTime - lastExecution) < executionStep + epsilon) {
            log.info("Delta too small. skipping");
            eligible = false;
        } else {
            eligible = true;
        }

        this.lastExecutions.put(hash, executionTime);
        this.cleanUp(executionTime);
        return eligible;
    }

    private void cleanUp(long executionTime) {
        int sizeBefore = lastExecutions.size();
        log.info(lastExecutions);
        Iterator<Integer> it = lastExecutions.keySet().iterator();
        while (it.hasNext()) {
            if (executionTime - lastExecutions.get(it.next()) > executionStep) {
                it.remove();
            }
        }

        int sizeAfter = lastExecutions.size();
        if (sizeBefore != sizeAfter) {
            log.info(String.format("Removed %d objects", sizeBefore - sizeAfter));
        }
    }

    private int getHash(RDFTuple row) {
        int i = 0;
        boolean done = false;
        StringBuilder result = new StringBuilder();
        // i hope noone ever sees this
        while (!done) {
            try {
                String entry = row.get(i);
                result.append(entry);
            } catch (IndexOutOfBoundsException e) {
                done = true;
            }
            ++i;
        }
        int hash = result.toString().hashCode();
        log.info("--HASH--");
        log.info(hash);
        return hash;
    }
}
