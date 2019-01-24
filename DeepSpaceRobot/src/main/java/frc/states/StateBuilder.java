package frc.states;

public class StateBuilder {

    private State current;

    public StateBuilder(State current) {
        this.current = current;
        current.initialize();
    }

    public void step() {
        State state = current.periodic();

        if (!current.equals(state)) {
            if (current != null) {
                current.finish();
            }

            state.initialize();
            current = state;
        }
    }

    public State getCurrentState() {
        return this.current;
    }

    public void setCurrentState(State current) {
        this.current = current;
    }

}