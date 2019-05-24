package frc.robot.state;

public interface State {

    void initialize();

    State periodic();

    void finish();
}