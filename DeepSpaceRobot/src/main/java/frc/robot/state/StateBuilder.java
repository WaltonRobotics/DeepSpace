package frc.robot.state;

import frc.robot.util.CrashDump;

public class StateBuilder {

    private State current;

    public StateBuilder(State current) throws IllegalArgumentException {
        this.current = current;

        if (current == null) {
            IllegalArgumentException e = new IllegalArgumentException("Make the first state not null!");

            CrashDump.logThrowableCrash(e);

            throw e;
        }

        current.initialize();
    }

    public void step() {
        State state = current.periodic();

        if (!current.equals(state)) {
            if (current != null) {
                current.finish();
            }

            if (state != null) {
                state.initialize();
            }
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