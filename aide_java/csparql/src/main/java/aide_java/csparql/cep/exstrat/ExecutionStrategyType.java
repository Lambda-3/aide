package aide_java.csparql.cep.exstrat;

public enum ExecutionStrategyType {
    
    Continuous(Continuous.class),

    NewOnly(NewOnly.class),
    
    Precise(Precise.class);
    
    private Class<? extends ExecutionStrategy> cls;
    
    ExecutionStrategyType(Class<? extends ExecutionStrategy> cls) {
        this.cls = cls;
    }
    
    Class<? extends ExecutionStrategy> getCorrespondingClass() {
        return cls;
    }
}
