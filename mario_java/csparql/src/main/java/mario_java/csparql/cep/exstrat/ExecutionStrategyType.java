package mario_java.csparql.cep.exstrat;

public enum ExecutionStrategyType {
    
    BasedOnExecutionTime(BasedOnExecutionTime.class);

    // Continuious,
    //
    // BasedOnExecutionTime
   
    private Class<? extends ExecutionStrategy> cls;
    
    ExecutionStrategyType(Class<? extends ExecutionStrategy> cls) {
        this.cls = cls;
    }
    
    Class<? extends ExecutionStrategy> getCorrespondingClass() {
        return cls;
    }
}
