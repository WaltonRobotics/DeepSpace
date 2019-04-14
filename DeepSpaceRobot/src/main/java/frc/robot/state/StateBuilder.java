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
    if (current != null) {
      State state = current.periodic();

      if (!current.equals(state)) {
        current.finish();

        if (state != null) {
          state.initialize();
        }
        current = state;
      }
    }
  }

  public State getCurrentState() {
    return current;
  }

  public void setCurrentState(State current) {
    this.current = current;
  }

  @Override
  public String toString() {
    return "StateBuilder{" +
        "current=" + current +
        '}';
  }

}