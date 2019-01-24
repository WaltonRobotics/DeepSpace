package frc.states;

public class State {

    private final GenericTransition initialize;
    private final PeriodicTransition periodic;
    private final GenericTransition finish;

    public State(GenericTransition initialize, PeriodicTransition periodic, GenericTransition finish) {
        this.initialize = initialize;
        this.periodic = periodic;
        this.finish = finish;
    }

    public State(PeriodicTransition periodic) {
        this(() -> null, periodic, () -> null);
    }

    public void initialize() {
        initialize.run();
    }

    public State periodic() {
        return periodic.run();
    }

    public void finish() {
        finish.run();
    }

}