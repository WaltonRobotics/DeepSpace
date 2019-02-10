package frc.robot.state;

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

    protected void initialize() {
        if (initialize != null) {
            initialize.run();
        }
    }

    protected State periodic() {
        if (periodic != null) {
            return periodic.run();
        }

        return null;
    }

    protected void finish() {
        if (finish != null) {
            finish.run();
        }
    }

    public GenericTransition getInitialize() {
        return initialize;
    }

    public PeriodicTransition getPeriodic() {
        return periodic;
    }

    public GenericTransition getFinish() {
        return finish;
    }

}