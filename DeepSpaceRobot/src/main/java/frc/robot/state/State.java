package frc.robot.state;

public abstract class State {
    protected abstract void initialize();

    protected abstract State periodic();

    protected abstract void finish();
}